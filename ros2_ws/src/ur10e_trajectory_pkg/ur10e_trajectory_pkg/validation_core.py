#Set 
#Rail is included as a full 7th DOF in every IK optimization (not a fallback).
#Collision, singularity, and rail position/velocity checks all apply during
#the transition period too when check_transition=True; joint jump checks
#never apply there (see process_matlab_validation's docstring for why).
import tempfile
import os
import numpy as np
import roboticstoolbox as rtb
import pybullet as pb
from spatialmath import SE3, UnitQuaternion
from spatialgeometry import Cuboid

from ur10e_trajectory_pkg.configurations import (
    ARM_SLICE,
    JOINT_NAMES,
    NUM_JOINTS,
    PERIODIC_JOINTS,
)
# The cap lives with the other limits; re-exported here for existing callers.
from ur10e_trajectory_pkg.motion_limits import (  # noqa: F401
    RAIL_VEL_SAFETY_CAP,
    SELF_CLEARANCE_FLOOR_M,
)

# How far self_clearance looks. Pairs farther apart than this report this
# distance, which is plenty above SELF_CLEARANCE_FLOOR_M and bounds the query.
SELF_CLEARANCE_QUERY_DISTANCE_M = 0.05
from ur10e_trajectory_pkg.joint_coordinates import (
    nearest_feasible_lift,
    winding_numbers,
)
from ur10e_trajectory_pkg.pose_metrics import (
    IK_ORIENTATION_TOL_RAD,
    IK_POSITION_TOL_M,
    pose_error,
)
from scipy.interpolate import PchipInterpolator
import matplotlib.pyplot as plt

EE_LINK = "tool0"

# RAIL_VEL_SAFETY_CAP is defined in motion_limits: a deliberate derate, and
# the validator enforces min(URDF, cap) for the rail (see __init__).

# Seed for the waypoint-recovery perturbations. Fixed so that the same input
# gives the same verdict: without it the retries drew from numpy's global
# generator, seeded from OS entropy, and three runs of one 500-waypoint file
# returned 364, 366 and 365 feasible waypoints.
#
# This makes results repeatable. It does NOT make them better -- the retry
# still perturbs within one solution basin and cannot reach another branch,
# so a false rejection is now a reproducible false rejection. Pass seed=None
# to restore unseeded exploration.
DEFAULT_IK_SEED = 20260913

# ikine_LM defaults to slimit=100: on failing to converge from q0 it retries
# from up to 99 further configurations of its own choosing, and with seed=None
# those are drawn nondeterministically. Measured on an unreachable target,
# five identical calls burned 100 searches each and returned five different
# configurations.
#
# Held at 1 so each call is exactly one deterministic search from the seed we
# supply. Retry policy belongs in _solve_waypoint_with_recovery, which already
# owns it; nesting a second, invisible search inside it made the attempt
# accounting meaningless and determinism unprovable.
#
# Note those internal restarts only ever fired on CONVERGENCE failure, never
# when a solution converged and was then rejected by our own condition-number,
# velocity or collision gates. That is the common case, so this removes far
# less exploration than the numbers suggest.
IK_SEARCH_LIMIT = 1

# Stopping threshold handed to ikine_LM. The solver minimises the QUADRATIC
# error E = 0.5 * e.T @ We @ e over the 6-vector angle-axis error e, and stops
# at E < tol. So this is not a bound on pose error directly: the previous
# value of 1e-4 admitted |e| up to sqrt(2e-4) = 0.0141, about 0.81 deg when
# angular error dominates.
#
# That explained the census distribution exactly: accepted orientation error
# had a median of 0.44 deg with a viability cliff between 0.25 and 0.5 deg.
# Those numbers described this stopping rule, not the task.
#
# 5e-7 is the value implied by a 1 mm translation bound under equal weighting,
# and is tighter than the roughly 1.5e-6 implied by 0.1 deg of orientation
# alone, so one scalar serves both. Measured over the 500-waypoint trajectory,
# against the old 1e-4:
#
#   orientation error, median   0.4395 deg  ->  0.0008 deg
#   waypoints within 1mm/0.1deg      249    ->  500
#   longest segment                  417    ->  419
#   median solver iterations           2    ->  2
#   tracking run time                0.4 s  ->  0.4 s
#
# Three orders of magnitude of orientation accuracy at no measurable cost, so
# there was never a reason to relax the acceptance limit to match the old
# distribution. Acceptance must still check position and orientation errors
# explicitly: this is a scalar on a weighted sum, not a bound on either.
IK_SOLVER_TOL = 5e-7

# Independent failure flags. Never collapsed: a configuration can miss its
# target AND collide, and knowing both is the difference between "the
# requested pose collides" and "somewhere else collides".
FAILURE_FLAGS = (
    'solver_failed',
    'pose_position_failed',
    'pose_orientation_failed',
    'collision',
    'singular',
    'arm_velocity_failed',
    'rail_velocity_failed',
)

# Order for the PRIMARY reason only; every set flag is reported.
#
# Pose mismatch outranks collision, singularity and velocity because those
# describe a configuration that is not a solution to the commanded target at
# all. Saying a trajectory is singular, when the configuration measured was
# metres from where it was asked to be, describes the wrong thing.
FAILURE_PRECEDENCE = (
    'solver_failed',
    'pose_position_failed',
    'pose_orientation_failed',
    'collision',
    'singular',
    'arm_velocity_failed',
    'rail_velocity_failed',
)


def format_failure(flags, details, suffix=''):
    """Render set flags as a human string.

    A formatter over the flags, deliberately not an if/elif classifier: the
    old one returned the first matching condition and discarded the rest, so
    simultaneous failures were invisible.
    """
    set_flags = [name for name in FAILURE_PRECEDENCE if flags.get(name)]
    if not set_flags:
        return f'Unknown failure{suffix}'

    primary = set_flags[0]
    if primary in ('pose_position_failed', 'pose_orientation_failed'):
        headline = f'POSE_MISMATCH: {details.get("pose", "")}'
        others = [f for f in set_flags
                  if f not in ('pose_position_failed', 'pose_orientation_failed')]
    else:
        headline = details.get(primary, primary)
        others = set_flags[1:]

    if others:
        headline += f' Also failed: {", ".join(others)}.'
    return headline + suffix

class TrajectoryValidator:
    def __init__(self, urdf_path, mesh_base_path=None, framerate=30,
                 seed=DEFAULT_IK_SEED, solver_tol=IK_SOLVER_TOL):
        self.framerate = framerate

        # Instance-owned generator, not numpy's global one, so validating a
        # trajectory cannot disturb random state elsewhere in the process.
        self._seed = seed
        self._rng = np.random.default_rng(seed)
        self._solver_tol = solver_tol

        # rtb.ERobot.URDF() (and pybullet's loadURDF) resolve package://
        # mesh URIs against their own default search paths -- NOT against
        # any folder named after the package sitting near the URDF file.
        # To point both at a real local mesh directory instead, rewrite
        # every 'package://ur_description/' URI to an absolute local path
        # before parsing, bypassing that resolution entirely. The SAME
        # rewritten URDF is fed to both roboticstoolbox (kinematics/IK)
        # and pybullet (collision), so there is exactly one source of
        # truth for geometry and no chance of the two backends disagreeing
        # about mesh placement.
        with open(urdf_path, 'r') as f:
            urdf_text = f.read()
        if mesh_base_path is not None:
            mesh_base_path = mesh_base_path.rstrip('/')
            urdf_text = urdf_text.replace('package://ur_description/', mesh_base_path + '/')

        # Written next to the original URDF (not into an arbitrary tmp
        # dir) so any *relative* mesh paths still in the URDF continue to
        # resolve the same way they did before.
        urdf_dir = os.path.dirname(os.path.abspath(urdf_path)) or '.'
        tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False, dir=urdf_dir)
        tmp.write(urdf_text)
        tmp.close()
        self._resolved_urdf_path = tmp.name 


        try:
            self.robot = rtb.ERobot.URDF(self._resolved_urdf_path)

            # Sanity-check that the ee link we're about to pin everywhere
            # actually exists in this robot's link set.
            ee_names = [l.name for l in self.robot.links]
            if EE_LINK not in ee_names:
                raise ValueError(
                    f"Expected end-effector link '{EE_LINK}' not found in URDF links: {ee_names}"
                )

            # Precompute a name -> link index map and a name -> parent-name
            # map once, for fast adjacency lookups during collision
            # checking.
            self._link_index_by_name = {l.name: idx for idx, l in enumerate(self.robot.links)}
            self._parent_name_by_index = {}
            for idx, link in enumerate(self.robot.links):
                parent = getattr(link, 'parent', None)
                self._parent_name_by_index[idx] = parent.name if parent is not None else None

            # Collapse chains of FIXED joints into single "rigid clusters"
            # before computing adjacency. e.g. rail_base_link
            # -[prismatic]-> base_link -[fixed]-> base_link_inertia:
            # base_link and base_link_inertia move together with zero
            # relative motion, so base_link_inertia is physically just as
            # "adjacent" to rail_base_link as base_link is. A plain
            # parent/child (1-hop) check misses this and reports a
            # false-positive self-collision between rail_base_link and
            # base_link_inertia. This mirrors what MATLAB's rigidBodyTree
            # effectively does for 'SkippedSelfCollisions','adjacent'.
            self._anchor_by_index = {}
            for idx in range(len(self.robot.links)):
                self._anchor_by_index[idx] = self._compute_anchor(idx)

            self._q_link_names = [l.name for l in self.robot.links if getattr(l, 'isjoint', False)]

            # Rail limits, for the position/velocity sanity checks in
            # _solve_waypoint_with_recovery. Verified against _q_link_names
            # (built from the SAME traversal order ikine_LM/jacobe/qlim all
            # use) rather than assumed -- fail loudly at startup if this
            # ever doesn't hold instead of silently reading the wrong
            # joint's limits later.
            if not self._q_link_names or self._q_link_names[0] != 'rail_carriage_link':
                raise ValueError(
                    f"Expected 'rail_carriage_link' to be the first actuated "
                    f"joint (q index 0), got order: {self._q_link_names}. "
                    f"Rail position/velocity checks assume q_full[0] is the rail."
                )
            self._rail_limits = (float(self.robot.qlim[0][0]), float(self.robot.qlim[1][0]))

            # The WHOLE ordering, not just the rail. Every per-joint vector in
            # this package -- limits, lifts, periodicity -- is indexed by
            # JOINT_NAMES, and a URDF that reordered two arm joints would
            # silently apply each one's limit to the other.
            self._joint_names = [
                getattr(self.robot.links[self._link_index_by_name[name]],
                        '_joint_name', None)
                for name in self._q_link_names
            ]
            if tuple(self._joint_names) != tuple(JOINT_NAMES):
                raise ValueError(
                    f'URDF joint order {self._joint_names} does not match '
                    f'JOINT_NAMES {list(JOINT_NAMES)}'
                )

            # Rail velocity ceiling, read from the URDF's <limit velocity="...">
            # via the same link the position limits above came from, then
            # clamped by the safety cap. This is the single source of truth for
            # what the rail can do -- previously three separate literals in this
            # file happened to agree, and the URDF's value was never read at all.
            rail_link = self.robot.links[self._link_index_by_name[self._q_link_names[0]]]
            rail_qdlim = getattr(rail_link, 'qdlim', None)
            self._rail_urdf_vel_limit = float(rail_qdlim) if rail_qdlim else None
            self._rail_vel_cap = RAIL_VEL_SAFETY_CAP
            self._rail_vel_limit = (min(float(rail_qdlim), RAIL_VEL_SAFETY_CAP)
                                    if rail_qdlim else RAIL_VEL_SAFETY_CAP)

            # Arm velocity ceilings, read the same way. These come from
            # Universal Robots: 120 deg/s for shoulder pan and lift, 180 deg/s
            # for the elbow and wrists, and our URDF carries them unmodified
            # from the original.
            #
            # They were previously ignored. A literal 2.0 rad/s, about 115
            # deg/s, was typed into three function signatures and applied
            # uniformly, holding the wrists to roughly two thirds of their
            # rated speed. The wrists are what performs a tumble, so that was
            # the largest artificial limit on how fast one could be
            # reproduced, and it came from no document at all.
            #
            # No safety cap here. The rail's cap is a deliberate derate on
            # hardware we have no trustworthy data for; the arm's limits are
            # the manufacturer's own.
            self._arm_vel_limits = np.array([
                float(getattr(self.robot.links[self._link_index_by_name[name]],
                              'qdlim', 0.0) or 0.0)
                for name in self._q_link_names[1:]
            ])
            if not np.all(self._arm_vel_limits > 0):
                raise ValueError(
                    'URDF does not declare a velocity limit for every arm '
                    f'joint: got {self._arm_vel_limits.tolist()}'
                )




            # # Kept for visualization/back-compat only (e.g. anything that
            # # still expects validator.env / validator.floor_box /
            # # validator.wall_box to exist) -- collision checking below no
            # # longer uses these.
            # self.wall_box = Cuboid(scale=[3.0, 0.05, 3.0], pose=SE3(0.0, 1.0, 0.0))
            # self.floor_box = Cuboid(scale=[3.0, 3.0, 0.05], pose=SE3(0.0, 0.0, -0.05))

            
            # # Matches rail_base_link's <collision><box size="2.0 0.1 0.05"/></collision>
            # # in the URDF (no <origin> offset there, so identity pose) -- for
            # # visualization only; the pybullet box built in
            # # _init_pybullet_collision_model is what's actually used for
            # # collision checking.
            # self.rail_box = Cuboid(scale=[2.0, 0.1, 0.05], pose=SE3())
            # self.env = [self.floor_box, self.wall_box, self.rail_box]

            self._init_pybullet_collision_model()
        finally:
            os.unlink(self._resolved_urdf_path)

    @property
    def velocity_limits(self):
        """Per-joint velocity ceilings in q_full order, straight from the URDF.

        The rail's is clamped by RAIL_VEL_SAFETY_CAP, a deliberate derate; the
        arm's are Universal Robots' own values, unmodified.
        """
        return np.concatenate(([self._rail_vel_limit], self._arm_vel_limits))

    def set_rail_velocity_cap(self, cap):
        """Replace RAIL_VEL_SAFETY_CAP for THIS instance, for sensitivity studies.

        Not an operating limit: the enforced rail velocity becomes
        min(URDF, cap), exactly as with the module cap, and effective_limits
        records the override so no artifact can pass it off as the default.
        """
        self._rail_vel_cap = float(cap)
        self._rail_vel_limit = (min(self._rail_urdf_vel_limit, self._rail_vel_cap)
                                if self._rail_urdf_vel_limit else self._rail_vel_cap)

    @property
    def rail_velocity_cap(self):
        return self._rail_vel_cap

    @property
    def urdf_velocity_limits(self):
        """The URDF's own values before any cap, None where it declares none.

        Recorded beside velocity_limits so an artifact shows both what the
        hardware description says and what was enforced.
        """
        return [self._rail_urdf_vel_limit] + self._arm_vel_limits.tolist()

    @property
    def joint_names(self):
        """Joint names in q_full order, as read from the URDF."""
        return tuple(self._joint_names)

    def _init_pybullet_collision_model(self):
        """Load the same URDF into a headless pybullet client, purely for
        collision queries, and build the link/joint index maps needed to
        drive it from a q_full vector and to translate its contact reports
        back into the roboticstoolbox link-name space that _is_adjacent()
        and the rest of this class already speak."""
        self._pb_client = pb.connect(pb.DIRECT)
        self.robot_id = pb.loadURDF(
            self._resolved_urdf_path,
            useFixedBase=True,
            flags=pb.URDF_USE_SELF_COLLISION | pb.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
            physicsClientId=self._pb_client,
        )

        base_name = pb.getBodyInfo(self.robot_id, physicsClientId=self._pb_client)[0].decode('utf-8')
        self._pb_link_name_by_index = {-1: base_name}
        pb_joint_index_by_link_name = {}
        for j in range(pb.getNumJoints(self.robot_id, physicsClientId=self._pb_client)):
            info = pb.getJointInfo(self.robot_id, j, physicsClientId=self._pb_client)
            child_link_name = info[12].decode('utf-8')
            joint_type = info[2]
            self._pb_link_name_by_index[j] = child_link_name
            if joint_type != pb.JOINT_FIXED:
                pb_joint_index_by_link_name[child_link_name] = j

        missing = [n for n in self._q_link_names if n not in pb_joint_index_by_link_name]
        if missing:
            raise ValueError(
                f"pybullet's URDF parse is missing movable joint(s) for link(s) {missing} "
                "-- roboticstoolbox and pybullet disagree about which joints are actuated "
                "in this URDF."
            )
        # Same order as self._q_link_names, i.e. same order as q_full.
        self._pb_joint_indices = [pb_joint_index_by_link_name[n] for n in self._q_link_names]

        # Wall: vertical plane parallel to XZ, standing at Y = +1.0m.
        # Floor: horizontal plane parallel to XY, lying at Z = -0.05m.
        # (Matches MATLAB's actual code, not its stale comment -- see script
        # header.) These two names were previously swapped, and the comment
        # naming their planes was wrong in both directions: a surface at
        # constant Y is parallel to XZ, not XY. Geometry is unchanged.
        #
        # Sizes below are the original Cuboid `scale` (full extents) halved,
        # since pybullet boxes take half-extents.
        wall_shape = pb.createCollisionShape(
            pb.GEOM_BOX, halfExtents=[3.0, 0.025, 1.5], physicsClientId=self._pb_client
        )
        floor_shape = pb.createCollisionShape(
            pb.GEOM_BOX, halfExtents=[3.0, 1.5, 0.025], physicsClientId=self._pb_client
        )
        # 6 m long, centred at X = +1.5, spanning -1.5 -> 4.5. Covers the
        # REACHABLE workspace rather than the rail's 0 -> 3 travel: the arm
        # overhangs each rail end by its own 1.3 m reach, and while these
        # planes were 3 m long it could dip below floor level there with no
        # floor to hit. Must stay in step with obstacle_markers.
        self.wall_id = pb.createMultiBody(
            baseCollisionShapeIndex=wall_shape,
            basePosition=[1.5, 1.0, 0.0],
            physicsClientId=self._pb_client,
        )
        self.floor_id = pb.createMultiBody(
            baseCollisionShapeIndex=floor_shape,
            basePosition=[1.5, 0.0, -0.05],
            physicsClientId=self._pb_client,
        )

        self._rail_link_names = {'rail_base_link', 'rail_carriage_link'}

    def __del__(self):
        client = getattr(self, '_pb_client', None)
        if client is not None:
            try:
                pb.disconnect(client)
            except Exception:
                pass

    def _compute_anchor(self, idx, _visited=None):
        """Walk up the parent chain through fixed (non-actuated) links only,
        stopping at the first actuated joint (or root). Returns that link's
        index -- links sharing an anchor move together as one rigid body."""
        if _visited is None:
            _visited = set()
        if idx in _visited:
            return idx  # guard against any accidental cycle
        _visited.add(idx)

        link = self.robot.links[idx]
        # isjoint True => this link carries an actuated DOF => it's its own anchor
        if getattr(link, 'isjoint', True):
            return idx

        parent_name = self._parent_name_by_index.get(idx)
        if parent_name is None or parent_name not in self._link_index_by_name:
            return idx  # root or parent not in our link set
        parent_idx = self._link_index_by_name[parent_name]
        return self._compute_anchor(parent_idx, _visited)

    def _is_adjacent(self, idx_i, idx_j):
        """True if link idx_i and idx_j are adjacent in the kinematic tree,
        after collapsing rigid (fixed-joint-only) clusters -- mirrors
        MATLAB's 'SkippedSelfCollisions','adjacent'. Two links are adjacent
        if they belong to the same rigid cluster, OR one cluster's anchor is
        the direct parent of the other cluster's anchor."""
        anchor_i = self._anchor_by_index[idx_i]
        anchor_j = self._anchor_by_index[idx_j]

        if anchor_i == anchor_j:
            return True

        def parent_cluster(anchor_idx):
            # The cluster "one hop up" from this anchor: the anchor of
            # THIS anchor's own immediate parent link. Comparing clusters
            # (not raw parent-name strings) is what makes this work even
            # when the parent link itself got absorbed into another
            # cluster via a fixed joint -- e.g. rail_carriage_link's real
            # parent is rail_base_link, but rail_base_link's OWN anchor is
            # 'world' (it's fixed to world), so a raw name comparison
            # against rail_base_link never matches; comparing against
            # rail_base_link's cluster (world) does.
            parent_name = self._parent_name_by_index.get(anchor_idx)
            if parent_name is None or parent_name not in self._link_index_by_name:
                return None
            parent_idx = self._link_index_by_name[parent_name]
            return self._anchor_by_index[parent_idx]

        return (parent_cluster(anchor_i) == anchor_j or
                parent_cluster(anchor_j) == anchor_i)

    def solve_ik_lm(self, target_pos, target_quat, q_seed_arm, rail_seed=0.0):
        """target_pos: [x, y, z]. target_quat: [x, y, z, w] (scipy/SE3 quat
        order, matching the rest of the pipeline -- get_end_effector_in_
        base_frame's q_B_G output is already in this order). q_seed_arm:
        6-element arm joint seed for the LM search.

        Returns: (rail_out, q_arm (6,), sol ) -- rail_out is the solved rail position

        FRAME CONTRACT -- target_pos/target_quat are in the URDF's ROOT frame
        ('world', which world-rail_base_joint fixes to rail_base_link at
        identity), NOT the arm base frame. ikine_LM is deliberately called
        without start=, so it solves the full 7-DOF chain from the root and
        interprets the target there.

        This is not a free choice: once the rail is a solved DOF the arm base
        rides the carriage, so it is not a fixed frame and cannot be the frame
        targets are expressed in. The rail base is the only static frame.

        KNOWN DISCREPANCY: ClientNode.get_end_effector_in_base_frame names its
        output frame 'B' after the UR base and builds targets relative to it.
        That frame and this one disagree by linear_rail_joint's <origin> (the
        mount height, 25 mm in Z and still a placeholder) plus the full rail
        position once the carriage moves. Reconcile once the mount height is
        measured -- p_B_I on the client side should mean the rail base in the
        arena, not the arm base.

        end=EE_LINK is required, not decorative. This URDF has three leaf
        links (ft_frame, base, tool0) and roboticstoolbox falls back to
        ee_links[0], which is ft_frame -- same position as tool0 but flipped
        180 degrees about X. Position would look right and orientation would
        be silently wrong, which is the half that matters for a tumble.
        """
        R_mat = UnitQuaternion(target_quat[3], target_quat[:3]).R  # spatialmath wants [w, x, y, z]
        T_target = SE3.Rt(R_mat, target_pos)


        q0 = np.concatenate(([rail_seed], q_seed_arm))
        sol = self.robot.ikine_LM(T_target, end=EE_LINK, q0=q0,
                                  mask=[1, 1, 1, 1, 1, 1], tol=self._solver_tol,
                                  slimit=IK_SEARCH_LIMIT, seed=self._seed)
        return sol.q[0],sol.q[1:], sol



    @staticmethod
    def _checked_configuration(q_full):
        """Reject anything that is not a complete, finite configuration.

        The defect this replaces was silent: a six-element arm vector passed
        to a seven-joint robot was padded by roboticstoolbox with a trailing
        zero, shifting every joint one position and pinning wrist_3 to zero.
        Nothing raised, and the reported condition number described a
        configuration the robot was not in.
        """
        q = np.asarray(q_full, dtype=float)
        if q.shape != (NUM_JOINTS,):
            raise ValueError(
                f'expected a full {NUM_JOINTS}-joint configuration '
                f'[rail_m, 6x arm_rad], got {q.size} values'
            )
        if not np.all(np.isfinite(q)):
            raise ValueError('configuration contains non-finite values')
        return q

    def compute_system_jacobian(self, q_full):
        """6x7 Jacobian of the tool, in the WORLD (URDF root) frame.

        Columns follow configurations.JOINT_NAMES: the rail first, then the
        six arm joints. Answers whether the rail-and-arm system together can
        produce a commanded Cartesian motion.

        Frame is named because it is part of the contract: a twist multiplied
        against this must be expressed in the same frame. Body-frame variants,
        if ever needed, get their own names rather than changing this one.
        """
        return self.robot.jacob0(self._checked_configuration(q_full), end=EE_LINK)

    def compute_arm_jacobian(self, q_full):
        """6x6 Jacobian of the six arm joints, in the WORLD frame.

        The arm columns of the system Jacobian, at the same configuration and
        in the same frame, so the two are directly comparable. Answers whether
        the UR arm itself is near a kinematic singularity, which the rail
        cannot rescue.

        Takes the COMPLETE configuration, not the six arm values: the rail
        position changes where the arm is, and the old six-value subchain call
        is exactly the shape that was silently padded.

        Selecting columns rather than calling the subchain is deliberate. Both
        give identical singular values, verified across 200 random
        configurations, so nothing is lost and there is one evaluation path.
        """
        return self.compute_system_jacobian(q_full)[:, ARM_SLICE]

    def _failure_reason(self, sol, res, condition_number_threshold,
                        max_rail_vel_threshold, max_joint_vel_threshold,
                        rail_fallback_exhausted):
        """Human string built from the flags, preserving every failure.

        The previous version was an if/elif chain returning the first match,
        so a configuration that both missed its target and collided was
        reported as one or the other. It also had no pose class at all.
        """
        suffix = (' (rail-assisted recovery also exhausted)'
                  if rail_fallback_exhausted else '')
        if res is None:
            return format_failure(
                {'solver_failed': True},
                {'solver_failed': f'IK did not converge (reason={sol.reason}).'},
                suffix)

        bad_arm = np.where(
            res['joint_vel'][1:] > max_joint_vel_threshold)[0].tolist()
        details = {
            'pose': (
                f"position {res['position_error_m']:.6f} m "
                f"{'>' if res['flags']['pose_position_failed'] else '<='} "
                f"{IK_POSITION_TOL_M:.6f} m; orientation "
                f"{np.rad2deg(res['orientation_error_rad']):.4f} deg "
                f"{'>' if res['flags']['pose_orientation_failed'] else '<='} "
                f"{np.rad2deg(IK_ORIENTATION_TOL_RAD):.4f} deg."
            ),
            'collision': (
                'Collision at reached target configuration: '
                f"q={np.round(res['q_full'], 3)}."
            ),
            'singular': (
                f"Singularity: condition number {res['cond_num']:.2f} > "
                f'{condition_number_threshold}.'
            ),
            'arm_velocity_failed': (
                f'Arm velocity exceeded {max_joint_vel_threshold} rad/s at '
                f'joint indices {bad_arm}.'
            ),
            'rail_velocity_failed': (
                f"Rail velocity {res['joint_vel'][0]:.3f} m/s exceeded "
                f'{max_rail_vel_threshold} m/s.'
            ),
        }
        return format_failure(res['flags'], details, suffix)

    def _solve_waypoint_with_recovery(self, target_pos, target_quat, seed_arm, rail_pos, prev_rail=None,
                                       prev_arm=None, check_jump=False,
                                       dt_waypoint=None,
                                       max_rail_vel_threshold=None,
                                       max_joint_vel_threshold=None,
                                       condition_number_threshold=50.0,
                                       max_rail_attempts=10,
                                       verbose=False, label='',
                                       recorder=None, record_context=None):
        """Solves IK for one waypoint with MATLAB-style local-perturbation
        retry, shared by process_matlab_validation (transition step AND
        main loop) and find_feasible_segments, so the recovery policy only
        lives in one place.


        check_jump=False; To be used only for transition-style solves, where the tight inter-waypoint dt
        threshold doesn't apply) skips the joint-velocity check entirely ; when True, prev_arm and dt_waypoint
        must be provided.

        Returns a dict: ok, q_arm, q_full, rail_pos, cond_num, joint_vel,
        attempts_used, rail_moved, reason (None if ok).
        """
        # None means 'use the rail's own limit', i.e. the URDF value clamped
        # by RAIL_VEL_SAFETY_CAP. Resolved here rather than in the signature
        # because it is per-instance -- it depends on the loaded model.
        if max_rail_vel_threshold is None:
            max_rail_vel_threshold = self._rail_vel_limit
        # Same treatment for the arm: None means the URDF's per-joint values.
        # A scalar is still accepted, as a deliberate uniform derate.
        if max_joint_vel_threshold is None:
            max_joint_vel_threshold = self._arm_vel_limits

        arm_limits = (self.robot.qlim[0][ARM_SLICE], self.robot.qlim[1][ARM_SLICE])
        arm_periodic = PERIODIC_JOINTS[ARM_SLICE]

        def lift_toward_reference(q_arm):
            """Put the solution on the turn nearest where we already are.

            The solver returns every revolute value wrapped into [-pi, pi],
            so a smooth motion across that boundary reads as a delta of
            nearly 2*pi. Applied BEFORE any velocity check, and the lifted
            values are what propagate onward, so the same continuous
            coordinates reach the next seed, the stored segment, the
            interpolation and the playback command.

            The reference is the previous COMMANDED configuration while
            tracking, and the caller's seed on a fresh entry -- never the
            perturbed numerical seed, which is an artefact of the retry loop.
            """
            reference = prev_arm if prev_arm is not None else seed_arm
            return nearest_feasible_lift(q_arm, reference, arm_limits,
                                         arm_periodic)

        def evaluate(q_arm_canonical, sol, rail):
            if not sol.success:
                return None
            q_arm = lift_toward_reference(q_arm_canonical)
            q_full = np.concatenate(([rail], q_arm))

            # Does this configuration actually reach the commanded pose?
            # ikine_LM's success flag measures convergence of its local
            # search, not distance to target, so it reports success from a
            # local minimum metres away. Checked first because a
            # configuration that misses its target is not a solution, whatever
            # else is true of it.
            position_err, orientation_err = pose_error(
                self.robot.fkine(q_full, end=EE_LINK), target_pos, target_quat)
            pose_position_failed = position_err > IK_POSITION_TOL_M
            pose_orientation_failed = orientation_err > IK_ORIENTATION_TOL_RAD

            J = self.compute_arm_jacobian(q_full)
            singular_values = np.linalg.svd(J, compute_uv=False)
            cond_num = (singular_values[0] / singular_values[-1]
                        if singular_values[-1] > 1e-9 else np.inf)
            low_cond = cond_num > condition_number_threshold

            # Joint and Rail Velocity Checks
            if check_jump:
                # Check Arm Velocities
                arm_vel = np.abs(q_arm - prev_arm) / dt_waypoint
                arm_jump = np.any(arm_vel > max_joint_vel_threshold)
                
                # Check Rail Velocity (safeguard against prev_rail being None)
                if prev_rail is not None:
                    rail_vel = np.abs(rail - prev_rail) / dt_waypoint
                    # print(f"\n rail_vel {rail_vel}\n dt={dt_waypoint}")
                    rail_jump = rail_vel > max_rail_vel_threshold
                else:
                    rail_vel = 0.0
                    rail_jump = False

                jump = arm_jump or rail_jump

                # Concatenate joint velocities for reporting: [rail_vel, arm_vel...]
                joint_vel = np.concatenate(([rail_vel if prev_rail is not None else 0.0], arm_vel))
            else:
                joint_vel = np.zeros(7)
                jump = arm_jump = rail_jump = False

            if len(q_full) != 7:
                raise ValueError(f"Expected q_full length 7, got {len(q_full)}")
                
            collide = self.check_all_collisions(q_full, verbose=verbose)
            flags = {
                'solver_failed': False,
                'pose_position_failed': bool(pose_position_failed),
                'pose_orientation_failed': bool(pose_orientation_failed),
                'collision': bool(collide),
                'singular': bool(low_cond),
                'arm_velocity_failed': bool(arm_jump),
                'rail_velocity_failed': bool(rail_jump),
            }
            return dict(q_arm=q_arm, q_arm_canonical=np.asarray(q_arm_canonical),
                        winding=winding_numbers(q_arm, q_arm_canonical).tolist(),
                        q_full=q_full, cond_num=cond_num, low_cond=low_cond,
                        jump=jump, collide=collide, joint_vel=joint_vel,
                        position_error_m=position_err,
                        orientation_error_rad=orientation_err,
                        pose_failed=bool(pose_position_failed
                                         or pose_orientation_failed),
                        flags=flags,
                        ok=not any(flags.values()))

        # True 7DOF rail solve
        search_radius = 0.05
        # max_rail_attempts + 1 solver calls: attempt 0 from the supplied
        # seed, then max_rail_attempts perturbed retries. This is the only
        # retry limit; a second parameter, max_attempts, was threaded through
        # the call chain but controlled nothing and merely inflated the
        # reported count by ten. Removed rather than revived, since the
        # two-phase arm-then-rail retry it once gated went away when the rail
        # became a solved degree of freedom.
        def emit(attempt, this_seed, new_rail, q_arm, sol, res):
            """Hand one attempt to an observer, without influencing the loop.

            Records every attempt including failed solves, and every gate as
            an INDEPENDENT boolean. evaluate() collapses the arm and rail
            velocity checks into one flag and returns None on solver failure,
            and _failure_reason applies precedence and describes only the last
            attempt, so none of those can reconstruct simultaneous failures.

            Pose errors are recorded raw, never thresholded here: stage 2b
            decides tolerances, and storing raw values means changing them
            later needs no re-run.
            """
            q_arm_used = res['q_arm'] if res is not None else np.asarray(q_arm)
            q_full = np.concatenate(([new_rail], q_arm_used))
            finite = bool(np.all(np.isfinite(q_full)))

            position_err = orientation_err = None
            cond = None
            singular_values = None
            if finite:
                position_err, orientation_err = pose_error(
                    self.robot.fkine(q_full, end=EE_LINK), target_pos, target_quat
                )
                sv = np.linalg.svd(self.compute_arm_jacobian(q_full),
                                   compute_uv=False)
                singular_values = sv.tolist()
                cond = float(sv[0] / sv[-1]) if sv[-1] > 1e-9 else float('inf')

            arm_violation = rail_violation = None
            arm_delta = arm_speed_out = rail_delta = None
            if check_jump and finite:
                # Raw signed delta kept alongside the speed. A revolute joint
                # solution is returned wrapped into [-pi, pi], so a smooth
                # motion crossing that boundary shows up as a delta near 2*pi:
                # a representation discontinuity, not a physical velocity. The
                # two are indistinguishable from the speed alone.
                delta = np.asarray(q_arm_used) - np.asarray(prev_arm)
                canonical_delta = np.asarray(q_arm) - np.asarray(prev_arm)
                arm_delta = delta.tolist()
                arm_speed = np.abs(delta) / dt_waypoint
                arm_speed_out = arm_speed.tolist()
                arm_violation = bool(np.any(arm_speed > max_joint_vel_threshold))
                if prev_rail is not None:
                    rail_delta = float(new_rail - prev_rail)
                    rail_speed = abs(rail_delta) / dt_waypoint
                    rail_violation = bool(rail_speed > max_rail_vel_threshold)
                else:
                    rail_violation = False

            recorder(dict(
                (record_context or {}),
                attempt=attempt,
                seed_arm=np.asarray(this_seed).tolist(),
                seed_rail=float(rail_pos),
                solver_success=bool(sol.success),
                solver_reason=str(getattr(sol, 'reason', '')),
                solver_searches=int(getattr(sol, 'searches', -1)),
                solver_iterations=int(getattr(sol, 'iterations', -1)),
                configuration_finite=finite,
                q_full=q_full.tolist() if finite else None,
                q_arm_canonical=np.asarray(q_arm).tolist(),
                winding=(None if res is None else res['winding']),
                rail_position=float(new_rail),
                position_error_m=position_err,
                orientation_error_rad=orientation_err,
                arm_condition_number=cond,
                arm_singular_values=singular_values,
                gate_solver_failed=not bool(sol.success),
                gate_pose_position=(
                    None if position_err is None
                    else bool(position_err > IK_POSITION_TOL_M)),
                gate_pose_orientation=(
                    None if orientation_err is None
                    else bool(orientation_err > IK_ORIENTATION_TOL_RAD)),
                gate_singular=(None if cond is None
                               else bool(cond > condition_number_threshold)),
                gate_arm_velocity=arm_violation,
                arm_delta_rad=arm_delta,
                arm_delta_canonical_rad=(
                    None if not check_jump or not finite
                    else canonical_delta.tolist()),
                arm_speed_rad_s=arm_speed_out,
                rail_delta_m=rail_delta,
                gate_rail_velocity=rail_violation,
                gate_collision=(None if res is None else bool(res['collide'])),
                accepted=bool(res is not None and res['ok']),
            ))

        for attempt in range(max_rail_attempts + 1):
            this_seed = seed_arm if attempt == 0 else (
                seed_arm + (2 * self._rng.random(6) - 1) * search_radius)
            new_rail, q_arm, sol = self.solve_ik_lm(target_pos, target_quat, this_seed, rail_seed=rail_pos)
            res = evaluate(q_arm, sol, new_rail)
            if recorder is not None:
                emit(attempt, this_seed, new_rail, q_arm, sol, res)
            if res is not None and res['ok']:
                if verbose:
                    print(f'{label}rail-assisted recovery succeeded '
                          f'(rail {rail_pos:.3f} -> {new_rail:.3f})')
                return dict(ok=True, q_arm=res['q_arm'], q_full=res['q_full'],
                            rail_pos=new_rail,
                            cond_num=res['cond_num'], joint_vel=res['joint_vel'],
                            attempts_used=attempt + 1, rail_moved=True, reason=None)
            last = (q_arm, sol, res)
            search_radius += 0.05

        q_arm, sol, res = last
        reason = self._failure_reason(sol, res, condition_number_threshold,max_rail_vel_threshold,
                                       max_joint_vel_threshold, rail_fallback_exhausted=True)
        return dict(ok=False,
                    q_arm=(res['q_arm'] if res else q_arm),
                    q_full=(res['q_full'] if res else None),
                    rail_pos=rail_pos, cond_num=(res['cond_num'] if res else None),
                    joint_vel=(res['joint_vel'] if res else None),
                    attempts_used=max_rail_attempts + 1, rail_moved=False, reason=reason)

    def check_all_collisions(self, q_full, verbose=False):
        """q_full: full joint vector in the same [rail, arm...] order as
        everywhere else in this file. Drives the pybullet model to this
        configuration and reports True on the first collision found --
        self-collision between non-adjacent links, or robot-vs-floor/wall,
        excluding the rail (which is expected to sit at/through the floor
        plane)."""
        q_full = np.asarray(q_full, dtype=np.float64)
        for pb_joint_idx, q_val in zip(self._pb_joint_indices, q_full):
            pb.resetJointState(self.robot_id, pb_joint_idx, float(q_val), physicsClientId=self._pb_client)

        pb.performCollisionDetection(physicsClientId=self._pb_client)

        # 1. Self-collision between non-adjacent robot tree links.
        self_contacts = pb.getContactPoints(
            bodyA=self.robot_id, bodyB=self.robot_id, physicsClientId=self._pb_client
        )
        for c in self_contacts:
            link_a, link_b = c[3], c[4]
            name_a = self._pb_link_name_by_index.get(link_a)
            name_b = self._pb_link_name_by_index.get(link_b)
            if name_a not in self._link_index_by_name or name_b not in self._link_index_by_name:
                continue
            idx_a = self._link_index_by_name[name_a]
            idx_b = self._link_index_by_name[name_b]
            if idx_a == idx_b or self._is_adjacent(idx_a, idx_b):
                continue
            if verbose:
                print(f"Self-collision: '{name_a}' vs '{name_b}'")
            return True

        # 2. Environment collisions (floor + wall), excluding the rail links.
        for env_id, env_label in ((self.floor_id, 'floor'), (self.wall_id, 'wall')):
            env_contacts = pb.getContactPoints(
                bodyA=self.robot_id, bodyB=env_id, physicsClientId=self._pb_client
            )
            for c in env_contacts:
                name_a = self._pb_link_name_by_index.get(c[3])
                if name_a in self._rail_link_names:
                    continue
                if verbose:
                    print(f"Env collision on link '{name_a}' vs {env_label}")
                return True

        return False

    def self_clearance(self, q_full, max_distance=SELF_CLEARANCE_QUERY_DISTANCE_M):
        """Smallest distance between non-adjacent robot links, and the pair.

        One closest-points query over the robot against itself, filtered by
        the same rule check_all_collisions applies to self-contacts: links
        known to the kinematic model, not the same link, not adjacent after
        collapsing rigid clusters. So pairs that overlap by design (the base
        inertia link inside the carriage) are skipped exactly as the boolean
        test skips them. Returns {'distance_m', 'links'}; distance_m is
        max_distance when no eligible pair is within it, and negative when
        links interpenetrate.
        """
        q_full = np.asarray(q_full, dtype=np.float64)
        for pb_joint_idx, q_val in zip(self._pb_joint_indices, q_full):
            pb.resetJointState(self.robot_id, pb_joint_idx, float(q_val),
                               physicsClientId=self._pb_client)
        best, pair = float(max_distance), None
        for point in pb.getClosestPoints(bodyA=self.robot_id, bodyB=self.robot_id,
                                         distance=max_distance,
                                         physicsClientId=self._pb_client):
            name_a = self._pb_link_name_by_index.get(point[3])
            name_b = self._pb_link_name_by_index.get(point[4])
            if (name_a not in self._link_index_by_name
                    or name_b not in self._link_index_by_name):
                continue
            idx_a = self._link_index_by_name[name_a]
            idx_b = self._link_index_by_name[name_b]
            if idx_a == idx_b or self._is_adjacent(idx_a, idx_b):
                continue
            if float(point[8]) < best:
                best, pair = float(point[8]), (name_a, name_b)
        return {'distance_m': best, 'links': pair}

    def reset_rng(self):
        """Return the recovery generator to its initial state.

        Called at the start of every top-level validation so that repeated
        requests to one long-lived validator are independent. Seeding at
        construction alone is not enough: generator state would carry from
        one request into the next, so the second validation of an identical
        trajectory would draw a different sequence and could reach a
        different verdict. Same failure shape as the start pose the server
        used to inherit from its own playback.
        """
        self._rng = np.random.default_rng(self._seed)

    def find_feasible_segments(self, ee_x, ee_y, ee_z, ee_quat, q_seed, min_length,
                                    dt_waypoint,
                                    max_rail_vel_threshold=None,
                                    max_joint_vel_threshold=None,
                                    condition_number_threshold=50.0,
                                    verbose=False, recorder=None):
            # Independent of any previous validation on this instance.
            self.reset_rng()

            # See _solve_waypoint_with_recovery for why this resolves here.
            if max_rail_vel_threshold is None:
                max_rail_vel_threshold = self._rail_vel_limit

            num_pts = len(ee_x)
            pos = np.column_stack((ee_x, ee_y, ee_z))
            quat = np.asarray(ee_quat)
            rail_pos = q_seed[0]  # running value -- may move if rail fallback ever fires
            q_home_arm = q_seed[1:]

            def try_point(target_pos, target_quat, seed_arm, seed_rail, prev_rail,
                          prev_arm, check_jump, waypoint_index=None):
                # entry_kind distinguishes the two ways a waypoint is solved.
                # A fresh entry starts a new segment from the caller's seed
                # after tracking broke; a continuation is seeded from the
                # previous waypoint's solution. Conflating them is what makes
                # "417 of 500" look like independent feasibility when it is
                # the length of one contiguous run.
                result = self._solve_waypoint_with_recovery(
                    target_pos, target_quat, np.asarray(seed_arm)[-6:], seed_rail, prev_rail=prev_rail,
                    prev_arm=(np.asarray(prev_arm)[-6:] if prev_arm is not None else None),
                    check_jump=check_jump, dt_waypoint=dt_waypoint,
                    max_rail_vel_threshold=max_rail_vel_threshold,
                    max_joint_vel_threshold=max_joint_vel_threshold,
                    condition_number_threshold=condition_number_threshold,
                    verbose=verbose, label='[segment scan] ',
                    recorder=recorder,
                    record_context=(None if recorder is None else dict(
                        waypoint_index=waypoint_index,
                        entry_kind='fresh' if not check_jump else 'continuation',
                    )),
                )
                return result['ok'], result['q_arm'], result['q_full'], result['rail_pos']

            segments = []
            current_start = None
            current_qs = []
            prev_config_arm = None
            idx = 0
            while idx < num_pts:
                target_pos = pos[idx, :]
                target_quat = quat[idx, :]
                if current_start is None:
                    ok, q_arm, q_full, rail = try_point(target_pos, target_quat, q_home_arm, rail_pos, None, None,
                                                         check_jump=False, waypoint_index=idx)
                    if ok:
                        current_start = idx
                        current_qs = [q_full]
                        prev_config_arm = q_arm
                        rail_prev = rail  # persists if this entry solve needed the rail fallback
                    elif verbose:

                        print(f'[segment scan] point {idx}: infeasible even as a fresh entry, skipping')
                else:
                    ok, q_arm, q_full, rail = try_point(target_pos, target_quat, q_arm, rail, rail_prev, prev_config_arm,
                                                         check_jump=True, waypoint_index=idx)
                    if ok:
                        current_qs.append(q_full)
                        prev_config_arm = q_arm
                        rail_prev = rail
                    else:
                        seg_len = len(current_qs)
                        if verbose:
                            print(f'[segment scan] run [{current_start}, {current_start + seg_len - 1}] '
                                f'ended at point {idx} (len={seg_len})')
                        if seg_len >= min_length:
                            segments.append({
                                'start_idx': current_start,
                                'end_idx': current_start + seg_len - 1,
                                'q_full': np.array(current_qs),
                                'length': seg_len,
                            })
                        current_start, current_qs, rail_prev,prev_config_arm = None, [], None, None
                        continue  # retry this same idx as a fresh potential start
                idx += 1

            if current_start is not None and len(current_qs) >= min_length:
                segments.append({
                    'start_idx': current_start,
                    'end_idx': current_start + len(current_qs) - 1,
                    'q_full': np.array(current_qs),
                    'length': len(current_qs),
                })

            return segments

    def process_feasible_segment(
            self,
            segment,
            q_start,
            dt_waypoint,
            t_transition=2.0,
            verbose=False,
        ):
            """Turn a single segment returned by find_feasible_segments into a
            full framerate trajectory, so it can be used as a drop-in
            replacement for process_matlab_validation's output (q_dot,
            q_interp) when the caller wants to run with "first feasible
            segment == the whole trajectory" instead of failing outright on
            any single infeasible waypoint.

            segment['q_full'] is already IK-solved/checked (that's what made it
            into the segment in the first place), so no solving happens here --
            this just does the same unchecked approach-phase prepend + pchip
            interpolation + np.gradient tail that process_matlab_validation
            does for the full waypoint set.

            q_start: the actual arm state to transition FROM (e.g. q_home).
            Prepended as an unchecked point at t=0, exactly like the
            transition-phase solve in process_matlab_validation -- the
            segment's own first point was already given an unchecked "entry"
            solve seeded from q_start's arm joints (see find_feasible_segments),
            so this only adds the initial physical approach from wherever the
            arm actually starts.
            dt_waypoint: must match what was passed into find_feasible_segments
            for this segment, so the recovered timing lines up.

            Returns: (q_dot, q_interp), matching the tail of the return tuple
            from process_matlab_validation.
            """
            q_full_seg = segment['q_full']
            seg_len = len(q_full_seg)

            t_traj = dt_waypoint * (seg_len - 1)
            t_final = t_traj + t_transition
            num_frames = int(t_final * self.framerate)

            config_soln = np.zeros((seg_len + 1, 7))
            config_soln[0, :] = q_start
            config_soln[1:, :] = q_full_seg

            t_waypoints_full = np.concatenate(
                ([0], np.linspace(t_transition, t_final, seg_len))
            )
            t_sim = np.linspace(0, t_final, num_frames)

            pchip_interpolator = PchipInterpolator(
                t_waypoints_full, config_soln, axis=0
            )
            q_interp = pchip_interpolator(t_sim)

            dt_sim = 1.0 / self.framerate
            q_dot = np.gradient(q_interp, dt_sim, axis=0)

            if verbose:
                print(
                    f'[segment interp] segment [{segment["start_idx"]}, {segment["end_idx"]}] '
                    f'(len={seg_len}) -> {num_frames} frames over {t_final:.2f}s'
                )

            # --- Plotting Joint Velocities ---

    # Host bind-mount directory for saving plots
            save_dir = '/root/ros2_ws/src/'
            os.makedirs(save_dir, exist_ok=True)

            arm_joint_labels = [
                'Shoulder Pan',
                'Shoulder Lift',
                'Elbow',
                'Wrist 1',
                'Wrist 2',
                'Wrist 3',
            ]

            # --- Plot 1: Arm Joint Velocities ---
            try:
                fig, ax = plt.subplots(figsize=(10, 6))
                for i in range(6):
                    ax.plot(
                        t_sim,
                        q_dot[:, i + 1],
                        linewidth=1.8,
                        label=arm_joint_labels[i],
                    )

                # The enforced per-joint limits, one pair of lines per
                # distinct value, rather than typed-in placeholders.
                for limit in np.unique(self._arm_vel_limits):
                    label = f'Arm limit {np.rad2deg(limit):.0f} deg/s'
                    ax.axhline(y=limit, color='r', linestyle='--', alpha=0.7,
                               label=label)
                    ax.axhline(y=-limit, color='r', linestyle='--', alpha=0.7)

                ax.set_title(
                    'Feasible Segment Arm Joint Velocities',
                    fontsize=14,
                    fontweight='bold',
                )
                ax.set_xlabel('Time (s)', fontsize=12)
                ax.set_ylabel('Velocity (rad/s)', fontsize=12)
                ax.grid(True, which='both', linestyle=':', alpha=0.6)
                ax.autoscale(enable=True, axis='both', tight=True)
                ax.legend(loc='upper right', bbox_to_anchor=(1.25, 1.0))

                arm_save_path = os.path.join(save_dir, 'arm_joint_velocities.png')
                plt.savefig(arm_save_path, bbox_inches='tight', dpi=300)
                plt.close(fig)

                if verbose:
                    print(f'Arm joint velocity plot saved to: {arm_save_path}')
            except Exception as e:
                print(f'Failed to generate arm velocity plot: {e}')

            # --- Plot 2: Rail Velocity ---
            try:
                fig, ax = plt.subplots(figsize=(10, 6))
                ax.plot(
                    t_sim,
                    q_dot[:, 0],
                    color='purple',
                    linewidth=2.0,
                    label='Rail Carriage',
                )

                ax.axhline(
                    y=self._rail_vel_limit,
                    color='m',
                    linestyle='--',
                    alpha=0.7,
                    label='Rail Limit (+)',
                )
                ax.axhline(
                    y=-self._rail_vel_limit,
                    color='m',
                    linestyle='--',
                    alpha=0.7,
                    label='Rail Limit (-)',
                )

                ax.set_title(
                    'Feasible Segment Rail Velocity', fontsize=14, fontweight='bold'
                )
                ax.set_xlabel('Time (s)', fontsize=12)
                ax.set_ylabel('Velocity (m/s)', fontsize=12)
                ax.grid(True, which='both', linestyle=':', alpha=0.6)
                ax.autoscale(enable=True, axis='both', tight=True)
                ax.legend(loc='upper right', bbox_to_anchor=(1.25, 1.0))

                rail_save_path = os.path.join(save_dir, 'rail_velocity.png')
                plt.savefig(rail_save_path, bbox_inches='tight', dpi=300)
                plt.close(fig)

                if verbose:
                    print(f'Rail velocity plot saved to: {rail_save_path}')
            except Exception as e:
                print(f'Failed to generate rail velocity plot: {e}')

            return q_dot, q_interp
    
    def process_task_segment(self, segment, dt_waypoint, verbose=False):
        """Controller-rate trajectory for a task that starts AT its first waypoint.

        No transition is prepended. The arm is brought to the first solved
        configuration by a separate warmup command, and the task itself starts
        from rest there (a spin-up), so the old unchecked 2 s home-to-waypoint
        window has nothing left to absorb. The segment must therefore begin at
        waypoint 0.

        Returns (q_dot, q_interp, t_sim), PCHIP through the solved
        configurations exactly as before, minus the prepended start.
        """
        if segment['start_idx'] != 0:
            raise ValueError(
                f"a task without a transition must start at waypoint 0; this "
                f"segment starts at {segment['start_idx']}")
        q_full = np.asarray(segment['q_full'], dtype=float)
        times = np.arange(len(q_full)) * float(dt_waypoint)
        count = max(2, int(round(times[-1] * self.framerate)) + 1)
        t_sim = np.linspace(0.0, times[-1], count)
        interpolator = PchipInterpolator(times, q_full, axis=0)
        q_interp = interpolator(t_sim)
        q_dot = interpolator.derivative(1)(t_sim)
        if verbose:
            print(f'[task] {len(q_full)} waypoints -> {count} frames over '
                  f'{times[-1]:.2f} s, no transition')
        return q_dot, q_interp, t_sim

    def process_matlab_validation(self, ee_x, ee_y, ee_z, ee_quat, q_start,
                                  max_rail_vel_threshold=None,
                                   max_joint_vel_threshold=None,
                                   condition_number_threshold=50.0,
                                   t_transition=2.0, t_traj=10.0,
                                   check_transition=False,
                                   verbose=False):
        """ee_quat: (N, 4) array of orientation targets, one per (ee_x,
        ee_y, ee_z) waypoint, in [x, y, z, w] order (matches SE3/scipy
        convention, and get_end_effector_in_base_frame's q_B_G output).

        check_transition=False (default): the home->first-waypoint solve
        is a single unchecked shot, matching the MATLAB script exactly --
        that jump is meant to be absorbed by the full t_transition window
        during pchip interpolation, not validated. This is the original,
        unmodified behavior; kept as a completely separate code path from
        the checked case below so turning this flag off is guaranteed
        identical to prior behavior.
        check_transition=True: the transition solve goes through the same
        collision/singularity/rail-fallback recovery as every other
        waypoint (joint-jump checking still never applies here -- it's
        calibrated for the tight inter-waypoint dt, not this multi-second
        window).

        If the rail ever has to move this way, that
        new position is carried forward as the locked rail_pos for every
        subsequent waypoint -- a real rail wouldn't snap back right after
        relocating.
        """
        # Independent of any previous validation on this instance.
        self.reset_rng()
        # None means 'use the rail's own limit', i.e. the URDF value clamped
        # by RAIL_VEL_SAFETY_CAP. Resolved here rather than in the signature
        # because it is per-instance -- it depends on the loaded model.
        if max_rail_vel_threshold is None:
            max_rail_vel_threshold = self._rail_vel_limit

        num_waypts = len(ee_x)
        t_final = t_traj + t_transition
        num_frames = int(t_final * self.framerate)

        # Running rail position -- starts at q_start's value, but may move
        # if a rail-assisted fallback ever fires
        rail_pos = q_start[0]
        q_arm_start = q_start[1:]

        pos_waypoints = np.column_stack((ee_x, ee_y, ee_z))
        quat_waypoints = np.asarray(ee_quat)
        config_soln = np.zeros((num_waypts + 1, 7))
        config_soln[0, :] = q_start

        dt_waypoint = (t_final - t_transition) / (num_waypts - 1)
        assert dt_waypoint==0.1, f'dt_waypoint={dt_waypoint} differs from SISFOS Default 0.1'
        # dt_waypoint= 0.1

        if check_transition:
            result = self._solve_waypoint_with_recovery(
                pos_waypoints[0, :], quat_waypoints[0, :], q_arm_start, rail_pos,
                prev_rail=None, prev_arm=None, check_jump=False, dt_waypoint=dt_waypoint,
                max_rail_vel_threshold=max_rail_vel_threshold,
                max_joint_vel_threshold=max_joint_vel_threshold,
                condition_number_threshold=condition_number_threshold,
                verbose=verbose, label='[transition] ')
            if not result['ok']:
                msg = f'Transition solve failed: {result["reason"]}'
                if verbose:
                    print(f'[transition] {msg}')
                return False, np.array([]), np.array([]), msg
            q_arm_transition = result['q_arm']
            rail_pos = result['rail_pos']
        else:
            # Original unchecked behavior, untouched.
            rail_pos ,q_arm_transition, sol0 = self.solve_ik_lm(
                pos_waypoints[0, :], quat_waypoints[0, :], q_arm_start,
                rail_seed=rail_pos)
            if not sol0.success:
                msg = f'IK did not converge on the approach/transition waypoint (target={np.round(pos_waypoints[0, :], 3)}, reason={sol0.reason})'
                if verbose:
                    print(f'[transition] {msg}')
                return False, np.array([]), np.array([]), msg

        config_soln[1, :] = np.concatenate(([rail_pos], q_arm_transition))
        prev_config_arm = q_arm_transition.copy()
        prev_rail=rail_pos.copy()

        # Checked loop: consecutive circle waypoints only (mirrors MATLAB's
        # `for k = 2:numWayPts`, which compares pos(2)..pos(numWayPts)
        # against each other -- pos(1) was already consumed above).
        for k in range(1, num_waypts):
            target_pos = pos_waypoints[k, :]
            target_quat = quat_waypoints[k, :]

            result = self._solve_waypoint_with_recovery(
                target_pos, target_quat, prev_config_arm, rail_pos,prev_rail=prev_rail,
                prev_arm=prev_config_arm, check_jump=True, dt_waypoint=dt_waypoint,
                max_rail_vel_threshold=max_rail_vel_threshold,
                max_joint_vel_threshold=max_joint_vel_threshold,
                condition_number_threshold=condition_number_threshold, 
                verbose=verbose, label=f'[waypoint {k}] ')

            if not result['ok']:
                msg = f'Waypoint {k} failed after {result["attempts_used"]} recovery attempts: {result["reason"]}'
                if verbose:
                    print(f'[waypoint {k}] {msg}')
                return False, np.array([]), np.array([]), msg

            config_soln[k + 1, :] = result['q_full']
            prev_config_arm = result['q_arm'].copy()
            rail_pos = result['rail_pos']  # persists if this waypoint needed the rail fallback

        t_waypoints_full = np.concatenate(([0], np.linspace(t_transition, t_final, num_waypts)))
        t_sim = np.linspace(0, t_final, num_frames)

        pchip_interpolator = PchipInterpolator(t_waypoints_full, config_soln, axis=0)
        q_interp = pchip_interpolator(t_sim)

        dt_sim = 1.0 / self.framerate
        q_dot = np.gradient(q_interp, dt_sim, axis=0)
        return True, q_dot, q_interp, 'Success'