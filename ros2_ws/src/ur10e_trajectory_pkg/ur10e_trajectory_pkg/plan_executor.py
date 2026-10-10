#!/usr/bin/env python3
"""Stream a certified best_plan.json to the robot as ROS velocity commands.

This replaces the Simulink refVel(t) -> Manual Switch -> 20 Hz hold chain.
Instead of a hand-written sinusoid, the reference is the plan itself: the
certified warmup and task curves, sampled on one clock for all seven joints.

Topics
  in   /joint_states                 measured 7-joint state (merger output)
  in   /rail/homed                   rail home-found flag (latched)
  out  /plan/joint_reference         reference position and velocity, always
                                     published, so a simulated robot can show
                                     the plan beside the real one
  out  /ur/joint_velocity_command    six arm rad/s, only while moving
  out  /rail/velocity_command        rail m/s, only while moving
  out  ~/phase                       idle / straighten arm / homing rail / ...

Services (std_srvs/Trigger)
  ~/start          one call from wherever the robot is, as a sequence:
                     1. straighten the arm to the plan's home posture, rail
                        held (only if the arm is off by <= auto_home_arm_limit;
                        otherwise refuse and name the pendant values)
                     2. home the rail (AXIS0 JOG HOME -1 via /rail_bridge/home)
                        when home_rail is 'always', or 'if_needed' and the
                        home-found flag is clear
                     3. move to the plan's home: rail only, arm held at home;
                        or, from the task start, retrace the certified warmup
                     4. the plan: warmup, or warmup + task (mode)
                   In mode 'full', a robot already at the task start (the
                   warmup was run before, e.g. with mode 'warmup') runs the
                   task directly instead.
                   Refuses unless /ur/status and /rail/status both say
                   'ready', and halts any motion if either stops saying so:
                   a faulted arm bridge never leaves the rail moving alone.
  ~/move_to_home   steps 1-3 only.
  ~/stop           zero all velocities now; aborts rail homing too.

Step 1 beyond auto_home_arm_limit (a robot left anywhere, e.g. at the end of
the task or where a halted run stopped) is collision-checked first: the
straight joint move to the home posture, rail held, is swept against the
planner's own model (walls, floor, self-clearance, joint limits) with
warmup.plan_warmup, the check the certified warmup passed. It runs only if
clear; otherwise ~/start refuses with the reason. The model knows nothing of
unmodelled lab obstacles. check_moves:=false (or a model that fails to load)
restores the plain limit: beyond it, move the arm from the pendant.

Steps 1 and 3's rail move are not certified motions, but the rail move keeps
the arm at the screened home posture, and the modelled wall and floor are
uniform along the rail, so it keeps the clearance that posture was screened
with. Rail homing is done in the same posture.

Command = reference velocity + kp * (reference - measured), the correction
clipped per joint. kp = 0 reproduces the Simulink model's pure feed-forward.
Execution aborts, commanding zero, if feedback goes stale or tracking error
exceeds the abort tolerances. time_scale < 1 slows the approach moves and the
warmup uniformly (see plan_reference); task_time_scale does the same for the
task alone (0, the default, means time_scale). The warmup's large swings need
a small scale; the task often runs at its recorded speed (1.0). The two are
separate stages, each checked against the speed limits at its own scale.
The bridges enforce their own speed caps and watchdogs independently of this
node.
"""

import time

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray, String
from std_srvs.srv import Trigger

from ur10e_trajectory_pkg import plan_artifact, plan_reference, trajectory_input
from ur10e_trajectory_pkg import preview_certified_plan_rviz as preview
from ur10e_trajectory_pkg.configurations import JOINT_NAMES, NUM_JOINTS, RAIL_INDEX
from ur10e_trajectory_pkg.ros_params import declare

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
HOME_RAIL_MODES = ('if_needed', 'always', 'never')


def per_joint(rail_value, arm_value):
    values = np.full(NUM_JOINTS, float(arm_value))
    values[RAIL_INDEX] = float(rail_value)
    return values


class ReferenceStage:
    def __init__(self, reference, label):
        self.reference, self.label = reference, label
        self.started = None


class HomeRailStage:
    label = 'homing rail'

    def __init__(self):
        self.started = None
        self.future = None
        self.accepted = False


class PlanExecutor(Node):

    def __init__(self):
        super().__init__('plan_executor')
        plan_path = declare(self, 'plan', '')
        csv_path = declare(self, 'csv', '')
        self.rate_hz = declare(self, 'rate_hz', 20.0)
        self.time_scale = declare(self, 'time_scale', 0.1)
        self.task_time_scale = declare(self, 'task_time_scale', 0.0) or self.time_scale
        mode = self.mode = declare(self, 'mode', 'warmup')
        self.settle_s = declare(self, 'settle_s', 1.0)
        self.kp = declare(self, 'kp', 0.0)
        self.feedback_timeout = declare(self, 'feedback_timeout', 0.25)
        self.speed_limit = per_joint(declare(self, 'rail_speed_limit', 0.05),
                                     declare(self, 'arm_speed_limit', 0.5))
        self.max_correction = per_joint(declare(self, 'rail_max_correction', 0.005),
                                        declare(self, 'arm_max_correction', 0.05))
        self.start_tolerance = per_joint(declare(self, 'rail_start_tolerance', 0.01),
                                         declare(self, 'arm_start_tolerance', 0.03))
        # Tight enough to hold the tool: at 0.15 rad per joint, the old value,
        # a stalled arm could leave the tool 27 cm off while the rail moved.
        self.abort_tolerance = per_joint(declare(self, 'rail_abort_tolerance', 0.01),
                                         declare(self, 'arm_abort_tolerance', 0.02))
        self.speed_scaling_check = declare(self, 'speed_scaling_check', True)
        self.approach_speed_fraction = declare(self, 'approach_speed_fraction', 0.8)
        self.auto_home_arm_limit = declare(self, 'auto_home_arm_limit', 0.2)
        self.home_rail = declare(self, 'home_rail', 'if_needed')
        self.check_moves = declare(self, 'check_moves', True)
        urdf = declare(self, 'urdf', '/root/ros2_ws/ur10e.urdf')
        self.homing_timeout = declare(self, 'homing_timeout', 250.0)
        if not plan_path:
            raise ValueError('the plan parameter (best_plan.json) is required')
        if mode not in ('warmup', 'full'):
            raise ValueError("mode must be 'warmup' or 'full'")
        if self.home_rail not in HOME_RAIL_MODES:
            raise ValueError(f'home_rail must be one of {HOME_RAIL_MODES}')

        self.plan = plan_artifact.load_plan(plan_path)
        csv_path = csv_path or self.plan['recording']['csv_path']
        trajectory = trajectory_input.load(csv_path)
        dt = preview.recording_step(csv_path, self.plan, trajectory=trajectory)
        # The same structural and rail-cap checks the RViz player applies.
        preview.playback_frames(self.plan, dt, 10.0)
        self.home = np.asarray(self.plan['home'], dtype=float)
        self.warmup_segments = plan_reference.plan_segments(
            self.plan, dt, self.settle_s, include_task=False)
        self.task_segments = plan_reference.task_only_segments(self.plan, dt, self.settle_s)
        self.task_start = np.asarray(self.plan['q_path'][0], dtype=float)
        warmup = plan_reference.Reference(self.warmup_segments, self.time_scale)
        task = plan_reference.Reference(self.task_segments, self.task_time_scale)
        self.warmup_peaks = warmup.peak_speeds(plan_rate_hz=200.0)
        self.task_peaks = task.peak_speeds(plan_rate_hz=200.0)
        self.get_logger().info(f'Loaded {plan_path}: mode {mode}, home_rail {self.home_rail}')
        stages = [('warmup', warmup, self.time_scale, self.warmup_peaks)]
        if mode == 'full':
            stages.append(('task', task, self.task_time_scale, self.task_peaks))
        for name, reference, scale, peaks in stages:
            self.get_logger().info(
                f'{name}: time_scale {scale}, {reference.duration:.1f} s wall clock; peak '
                f'speeds rail {peaks[0]:.4f} m/s, arm {np.round(peaks[1:], 3).tolist()} rad/s')
            if self.over_limit(peaks) is not None:
                self.get_logger().warn(f'{name}: {self.over_limit(peaks)}; ~/start will refuse '
                                       f'to run it')

        self.validator = self.load_collision_model(urdf) if self.check_moves else None

        self.measurement = None         # (pose, velocity, monotonic stamp)
        self.rail_homed = None
        self.stage = None
        self.pending = []
        self.sequence = None
        self.hold = self.home.copy()
        self.phase = 'idle'

        # Feedback keeps arriving while a service or the tick is busy (the
        # collision check takes about a second), so a stage that starts
        # right after it sees fresh state. Services and the tick share the
        # node's default group and never run concurrently with each other.
        sensors = ReentrantCallbackGroup()
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10,
                                 callback_group=sensors)
        self.device_seen = {'arm (/ur/joint_states)': None, 'rail (/rail/joint_state)': None}
        self.create_subscription(JointState, '/ur/joint_states',
                                 lambda _: self.saw('arm (/ur/joint_states)'), 10,
                                 callback_group=sensors)
        self.create_subscription(JointState, '/rail/joint_state',
                                 lambda _: self.saw('rail (/rail/joint_state)'), 10,
                                 callback_group=sensors)
        self.create_subscription(Bool, '/rail/homed', self.on_rail_homed, LATCHED,
                                 callback_group=sensors)
        # Each bridge says 'ready', or why its device would not follow.
        self.device_status = {'arm': None, 'rail': None}
        for device in self.device_status:
            self.create_subscription(
                String, f'/{"ur" if device == "arm" else "rail"}/status',
                lambda message, device=device: self.device_status.__setitem__(
                    device, message.data), LATCHED, callback_group=sensors)
        # Published by ur_bridge only while a UR program is playing; readings
        # older than a second mean no program is playing and are ignored.
        self.speed_scaling, self.speed_scaling_at = None, 0.0
        self.create_subscription(Float64, '/ur/speed_scaling', self.on_speed_scaling, 10,
                                 callback_group=sensors)
        self.last_tracking_log = 0.0
        self.reference_pub = self.create_publisher(JointState, '/plan/joint_reference', 10)
        self.arm_pub = self.create_publisher(Float64MultiArray, '/ur/joint_velocity_command', 10)
        self.rail_pub = self.create_publisher(Float64, '/rail/velocity_command', 10)
        self.phase_pub = self.create_publisher(String, '~/phase', 10)
        self.home_client = self.create_client(Trigger, '/rail_bridge/home')
        self.create_service(Trigger, '~/start', self.on_start)
        self.create_service(Trigger, '~/move_to_home', self.on_move_to_home)
        self.create_service(Trigger, '~/stop', self.on_stop)
        self.create_timer(1.0 / self.rate_hz, self.tick)

    # --- feedback ------------------------------------------------------------

    def on_joint_states(self, message):
        positions = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity))
        if all(name in positions for name in JOINT_NAMES):
            measured = np.array([positions[name] for name in JOINT_NAMES])
            velocity = np.array([velocities.get(name, 0.0) for name in JOINT_NAMES])
            # One assignment, so another thread never pairs a new pose with
            # an old stamp (see fresh_measurement).
            self.measurement = (measured, velocity, time.monotonic())

    def on_speed_scaling(self, message):
        self.speed_scaling, self.speed_scaling_at = message.data, time.monotonic()

    def slowed_by_scaling(self):
        """The current UR speed scaling if it is recent and below 100%, else None."""
        if (not self.speed_scaling_check or self.speed_scaling is None
                or time.monotonic() - self.speed_scaling_at > 1.0):
            return None
        return self.speed_scaling if self.speed_scaling < 0.99 else None

    def on_rail_homed(self, message):
        self.rail_homed = bool(message.data)

    def saw(self, device):
        self.device_seen[device] = time.monotonic()

    def stale_report(self):
        """Why /joint_states is not fresh, naming the device that went quiet."""
        now = time.monotonic()
        ages = ', '.join(f'{name} {"never seen" if at is None else f"{now - at:.2f} s ago"}'
                         for name, at in self.device_seen.items())
        return f'no fresh /joint_states (last messages: {ages})'

    def not_ready(self, allow_rail_homing=False):
        """Why the arm or rail would not follow commands, or None when both would."""
        problems = []
        for device, status in self.device_status.items():
            if status == 'ready' or (device == 'rail' and allow_rail_homing
                                     and status == 'homing'):
                continue
            problems.append(f'{device} bridge not ready: '
                            f'{status or "no status received (bridge not running?)"}')
        return '; '.join(problems) or None

    def fresh_measurement(self):
        measurement = self.measurement
        if measurement is None or time.monotonic() - measurement[2] > self.feedback_timeout:
            return None
        return measurement[0].copy()

    # --- sequence construction ----------------------------------------------

    def over_limit(self, peaks):
        over = peaks + self.max_correction > self.speed_limit
        if not np.any(over):
            return None
        names = [JOINT_NAMES[i] for i in np.flatnonzero(over)]
        return (f'speed limits exceeded on {names} (peaks {np.round(peaks, 4).tolist()}); '
                f'lower its time scale')

    def load_collision_model(self, urdf):
        """The planner's validator (walls, floor, self), or None if unavailable."""
        started = time.monotonic()
        try:
            from ament_index_python.packages import get_package_share_directory
            from ur10e_trajectory_pkg import planning_runtime
            validator = planning_runtime.make_validator(
                urdf, get_package_share_directory('ur_description'))
        except Exception as error:  # a missing model must not stop execution
            self.get_logger().warn(
                f'collision model unavailable ({error!r}): an arm more than '
                f'{self.auto_home_arm_limit} rad from home must be moved from the pendant')
            return None
        self.get_logger().info(f'collision model loaded from {urdf} in '
                               f'{time.monotonic() - started:.1f} s: arm moves to the home '
                               'posture from any pose are checked before they run')
        return validator

    def arm_far_from_home(self, measured):
        return bool(np.any(np.abs(measured[1:] - self.home[1:]) > self.auto_home_arm_limit))

    def check_return(self, measured):
        """None if the straight arm move to the home posture is clear, else why not.

        The move checked is the one straighten/approach will play: the arm
        joints to plan home, rail held. Its shape does not depend on timing,
        so checking it at the planner's own speed covers any slower playback.
        An unreferenced rail reads meaningless positions; the modelled wall
        and floor are uniform along the rail, so it is checked at plan home.
        """
        if self.validator is None:
            return 'no collision model loaded'
        from ur10e_trajectory_pkg import warmup
        start = measured.copy()
        if self.rail_homed is not True:
            start[RAIL_INDEX] = self.home[RAIL_INDEX]
        goal = start.copy()
        goal[1:] = self.home[1:]
        started = time.monotonic()
        result = warmup.plan_warmup(self.validator, start, goal)
        if result['status'] != warmup.OK:
            return f"{result['status']}: {result.get('reason')}"
        self.get_logger().info(
            f'checked the arm move to the home posture in {time.monotonic() - started:.1f} s: '
            f"clear of the modelled wall and floor, self-clearance "
            f"{result['min_self_clearance_m'] * 1000:.0f} mm")
        return None

    def far_refusal(self, measured, problem):
        offset = np.abs(measured[1:] - self.home[1:])
        worst = int(np.argmax(offset))
        return (f'arm is {offset[worst]:.3f} rad from plan home on joint {worst + 1} and the '
                f'move there was not run ({problem}); move it to plan home from the pendant '
                f'(degrees {np.round(np.degrees(self.home[1:]), 2).tolist()})')

    def approach_speed(self, time_scale=None):
        """Wall-clock approach speed limits expressed in plan time."""
        return self.approach_speed_fraction * self.speed_limit / (time_scale or self.time_scale)

    def task_stage(self, label='task'):
        return ReferenceStage(
            plan_reference.Reference(self.task_segments, self.task_time_scale), label)

    def straighten_arm(self, measured):
        goal = measured.copy()
        goal[1:] = self.home[1:]
        segment = plan_reference.move_segment(measured, goal, self.approach_speed(),
                                              name='straighten arm')
        return ReferenceStage(plan_reference.Reference([segment], self.time_scale),
                              'straighten arm')

    def approach(self, measured, include_plan):
        if (include_plan and self.mode == 'full' and
                plan_reference.at_task_start(self.plan, measured, self.start_tolerance)):
            # The warmup has already been run (e.g. mode:=warmup, then
            # mode:=full): start the task from here instead of going home.
            label = 'at the task start, warmup already done: running the task'
            if np.all(np.abs(measured - self.task_start) <= self.start_tolerance):
                return self.task_stage(label)
            align = plan_reference.move_segment(
                measured, self.task_start, self.approach_speed(self.task_time_scale),
                minimum_duration=1.0, name='align to task start')
            return ReferenceStage(plan_reference.Reference(
                [align] + self.task_segments, self.task_time_scale), label)
        arm_limit = self.auto_home_arm_limit
        if (self.arm_far_from_home(measured)
                and not plan_reference.at_task_start(self.plan, measured, self.start_tolerance)):
            problem = self.check_return(measured)
            if problem:
                raise ValueError(self.far_refusal(measured, problem))
            arm_limit = np.inf
        segments, description = plan_reference.approach_segments(
            self.plan, measured, self.start_tolerance, arm_limit,
            self.approach_speed(), self.settle_s)
        if include_plan:
            segments = segments + self.warmup_segments
            description = f'{description}, then the warmup'
        stages = []
        if segments:
            stages.append(ReferenceStage(
                plan_reference.Reference(segments, self.time_scale), description))
        if include_plan and self.mode == 'full':
            stages.append(self.task_stage())
        return stages or None

    def build_sequence(self, include_plan):
        """Stage builders from the current state, or a refusal message."""
        measured = self.fresh_measurement()
        if measured is None:
            return None, self.stale_report()
        if self.not_ready():
            return None, self.not_ready()
        if include_plan:
            # Only the stages that will run: from the task start, the task.
            skip_warmup = self.mode == 'full' and plan_reference.at_task_start(
                self.plan, measured, self.start_tolerance)
            checks = [] if skip_warmup else [('warmup', self.warmup_peaks)]
            if self.mode == 'full':
                checks.append(('task', self.task_peaks))
            for name, peaks in checks:
                if self.over_limit(peaks):
                    return None, f'{name}: {self.over_limit(peaks)}'
        if self.rail_homed is None and self.home_rail != 'never':
            return None, ("rail home-found flag unknown; if you homed it manually, "
                          "relaunch with home_rail:=never")
        if self.rail_homed is False and self.home_rail == 'never':
            return None, 'rail is not homed and home_rail is never'
        builders = []
        needs_homing = (self.home_rail == 'always' or
                        (self.home_rail == 'if_needed' and self.rail_homed is False))
        if needs_homing:
            arm_offset = np.abs(measured[1:] - self.home[1:])
            if self.arm_far_from_home(measured):
                problem = self.check_return(measured)
                if problem:
                    return None, self.far_refusal(measured, problem)
            if np.any(arm_offset > self.start_tolerance[1:]):
                builders.append(self.straighten_arm)
            builders.append(lambda _: HomeRailStage())
        builders.append(lambda q: self.approach(q, include_plan))
        # Build the first stage now, so its refusal reaches the caller.
        try:
            first, rest = self.split(builders[0](measured))
        except ValueError as error:
            return None, str(error)
        return [first] + rest + builders[1:], None

    @staticmethod
    def split(built):
        """A builder's result (None, a stage, or a list of stages) as the stage
        to run next and builders that hand over the rest, in order."""
        stages = built if isinstance(built, list) else [] if built is None else [built]
        if not stages:
            return None, []
        return stages[0], [lambda _, stage=stage: stage for stage in stages[1:]]

    def launch(self, include_plan, label, response):
        if self.stage is not None or self.pending:
            response.success, response.message = False, f'already running {self.sequence}'
            return response
        stages, refusal = self.build_sequence(include_plan)
        scaling = self.slowed_by_scaling()
        if not refusal and scaling is not None:
            refusal = (f'UR speed scaling is {scaling:.2f}: the arm would run at '
                       f'{scaling * 100:.0f}% of commanded speed while the rail '
                       'runs at 100%. Set the pendant speed slider to 100% (and leave reduced '
                       'mode). Override only for tests: speed_scaling_check:=false')
        if refusal:
            response.success, response.message = False, refusal
            return response
        first, self.pending = stages[0], stages[1:]
        if first is None and not self.pending:
            response.success, response.message = True, 'already at plan home'
            return response
        self.sequence = label
        problem = self.activate(first)
        if problem:
            self.pending, self.sequence = [], None
            response.success, response.message = False, problem
            return response
        response.success = True
        response.message = f'{label}: {first.label}'
        if self.pending:
            response.message += f' (then {len(self.pending)} more stage(s))'
        self.get_logger().info(response.message)
        return response

    def on_start(self, request, response):
        return self.launch(True, 'start', response)

    def on_move_to_home(self, request, response):
        return self.launch(False, 'move to home', response)

    def on_stop(self, request, response):
        self.halt('stop requested')
        # Also when idle: a stop request must always put zeros on the wire.
        self.publish_commands(np.zeros(NUM_JOINTS))
        response.success, response.message = True, 'stopped'
        return response

    # --- execution -----------------------------------------------------------

    def activate(self, stage):
        """Start a stage; return a problem string instead if it may not start."""
        if stage is None:
            return None
        if isinstance(stage, ReferenceStage):
            measured = self.fresh_measurement()
            if measured is None:
                return self.stale_report()
            problem = self.over_limit(stage.reference.peak_speeds(plan_rate_hz=200.0))
            if problem:
                return f'{stage.label}: {problem}'
            error = np.abs(measured - stage.reference.start)
            if np.any(error > self.start_tolerance):
                names = [f'{JOINT_NAMES[i]} off by {error[i]:.4f}'
                         for i in np.flatnonzero(error > self.start_tolerance)]
                return f'robot is not at the start of {stage.label}: {names}'
        elif not self.home_client.service_is_ready():
            return '/rail_bridge/home is not available'
        stage.started = time.monotonic()
        self.stage = stage
        self.get_logger().info(f'stage: {stage.label}')
        return None

    def next_stage(self):
        self.stage = None
        while self.pending:
            builder = self.pending.pop(0)
            measured = self.fresh_measurement()
            if measured is None:
                self.halt(self.stale_report())
                return
            try:
                stage, rest = self.split(builder(measured))
            except ValueError as error:
                self.halt(str(error))
                return
            if stage is None:
                continue
            self.pending = rest + self.pending
            problem = self.activate(stage)
            if problem:
                self.halt(problem)
            return
        self.get_logger().info(f'{self.sequence} complete')
        self.sequence = None

    def publish_commands(self, velocities):
        self.arm_pub.publish(Float64MultiArray(data=[float(v) for v in velocities[1:]]))
        self.rail_pub.publish(Float64(data=float(velocities[RAIL_INDEX])))

    def halt(self, reason):
        if self.stage is None and not self.pending:
            return
        self.pending = []
        self.publish_commands(np.zeros(NUM_JOINTS))
        measured = self.fresh_measurement()
        self.hold = measured if measured is not None else self.hold
        self.get_logger().warn(f'{self.sequence} halted: {reason}')
        self.stage, self.sequence = None, None

    def publish_reference(self, q, qd):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(JOINT_NAMES)
        message.position = np.asarray(q, dtype=float).tolist()
        message.velocity = np.asarray(qd, dtype=float).tolist()
        self.reference_pub.publish(message)

    def set_phase(self, phase):
        if phase != self.phase:
            self.phase = phase
            self.get_logger().info(f'phase: {phase}')
        self.phase_pub.publish(String(data=phase))

    def tick(self):
        if self.stage is None:
            self.set_phase('idle')
            self.publish_reference(self.hold, np.zeros(NUM_JOINTS))
            return
        if isinstance(self.stage, HomeRailStage):
            self.tick_homing(self.stage)
        else:
            self.tick_reference(self.stage)

    def tick_homing(self, stage):
        self.set_phase('homing rail')
        measured = self.fresh_measurement()
        if measured is None:
            self.halt(self.stale_report())
            return
        if self.not_ready(allow_rail_homing=True):
            self.halt(self.not_ready(allow_rail_homing=True))
            return
        # The rail's position is meaningless until homing completes: show the
        # measured pose rather than a reference.
        self.hold = measured
        self.publish_reference(measured, np.zeros(NUM_JOINTS))
        if time.monotonic() - stage.started > self.homing_timeout:
            self.halt('homing did not complete in time')
            return
        if stage.future is None:
            stage.future = self.home_client.call_async(Trigger.Request())
            return
        if not stage.accepted:
            if not stage.future.done():
                return
            result = stage.future.result()
            if result is None or not result.success:
                self.halt(f"homing refused: {getattr(result, 'message', 'no response')}")
                return
            stage.accepted = True
            # The bridge clears the flag before replying; do not let an old,
            # latched True end this homing before its False arrives.
            self.rail_homed = False
            return
        if self.rail_homed:
            self.get_logger().info(f'rail homed at {measured[RAIL_INDEX]:.4f} m')
            self.next_stage()

    def tick_reference(self, stage):
        reference = stage.reference
        elapsed = time.monotonic() - stage.started
        q_ref, qd_ref, phase = reference.sample(elapsed)
        self.publish_reference(q_ref, qd_ref)
        self.set_phase(phase)
        measured = self.fresh_measurement()
        if measured is None:
            self.halt(self.stale_report())
            return
        if self.not_ready():
            self.halt(self.not_ready())
            return
        scaling = self.slowed_by_scaling()
        if scaling is not None:
            self.halt(f'UR speed scaling dropped to {scaling:.2f}')
            return
        error = q_ref - measured
        if np.any(np.abs(error) > self.abort_tolerance):
            worst = int(np.argmax(np.abs(error) / self.abort_tolerance))
            self.halt(f'tracking error {error[worst]:.4f} on {JOINT_NAMES[worst]}')
            return
        self.log_tracking(error, qd_ref)
        command = qd_ref + np.clip(self.kp * error, -self.max_correction, self.max_correction)
        if np.any(np.abs(command) > self.speed_limit):
            self.halt(f'command {np.round(command, 4).tolist()} exceeds speed limits')
            return
        if elapsed >= reference.duration:
            self.publish_commands(np.zeros(NUM_JOINTS))
            self.hold = reference.end
            self.next_stage()
            return
        self.publish_commands(command)

    def log_tracking(self, error, qd_ref):
        """Once a second: tracking error, and how fast the arm moves vs. commanded."""
        now = time.monotonic()
        if now - self.last_tracking_log < 1.0:
            return
        self.last_tracking_log = now
        arm = int(np.argmax(np.abs(error[1:]))) + 1
        text = (f'tracking: rail {error[RAIL_INDEX] * 1000:+.1f} mm, worst arm '
                f'{error[arm]:+.4f} rad ({JOINT_NAMES[arm]})')
        reference = qd_ref[1:]
        measurement = self.measurement
        if measurement is not None and np.dot(reference, reference) > 1e-6:
            ratio = np.dot(measurement[1][1:], reference) / np.dot(reference, reference)
            text += f', arm speed {ratio * 100:.0f}% of reference'
        self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = PlanExecutor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # A signal can shut ROS down mid-spin; only real errors propagate.
        if rclpy.ok():
            raise
    finally:
        if node.stage is not None:
            node.publish_commands(np.zeros(NUM_JOINTS))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
