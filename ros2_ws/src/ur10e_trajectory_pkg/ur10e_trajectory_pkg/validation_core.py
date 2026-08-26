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
from scipy.interpolate import PchipInterpolator

EE_LINK = "tool0"


class TrajectoryValidator:
    def __init__(self, urdf_path, mesh_base_path=None, framerate=30):
        self.framerate = framerate

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

            # Kept for visualization/back-compat only (e.g. anything that
            # still expects validator.env / validator.floor_box /
            # validator.wall_box to exist) -- collision checking below no
            # longer uses these.
            self.floor_box = Cuboid(scale=[3.0, 0.05, 3.0], pose=SE3(0.0, 1.0, 0.0))
            self.wall_box = Cuboid(scale=[3.0, 3.0, 0.05], pose=SE3(0.0, 0.0, -0.05))

            
            # Matches rail_base_link's <collision><box size="2.0 0.1 0.05"/></collision>
            # in the URDF (no <origin> offset there, so identity pose) -- for
            # visualization only; the pybullet box built in
            # _init_pybullet_collision_model is what's actually used for
            # collision checking.
            self.rail_box = Cuboid(scale=[2.0, 0.1, 0.05], pose=SE3())
            self.env = [self.floor_box, self.wall_box, self.rail_box]

            self._init_pybullet_collision_model()
        finally:
            os.unlink(self._resolved_urdf_path)

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

        # Floor parallel to XY plane at Y = +1.0m, Wall parallel to XZ
        # plane at Z = -0.05m (matches MATLAB's actual code, not its stale
        # comment -- see script header). Sizes below are the original
        # Cuboid `scale` (full extents) halved, since pybullet boxes take
        # half-extents.
        floor_shape = pb.createCollisionShape(
            pb.GEOM_BOX, halfExtents=[1.5, 0.025, 1.5], physicsClientId=self._pb_client
        )
        wall_shape = pb.createCollisionShape(
            pb.GEOM_BOX, halfExtents=[1.5, 1.5, 0.025], physicsClientId=self._pb_client
        )
        self.floor_id = pb.createMultiBody(
            baseCollisionShapeIndex=floor_shape,
            basePosition=[0.0, 1.0, 0.0],
            physicsClientId=self._pb_client,
        )
        self.wall_id = pb.createMultiBody(
            baseCollisionShapeIndex=wall_shape,
            basePosition=[0.0, 0.0, -0.05],
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
        """
        R_mat = UnitQuaternion(target_quat[3], target_quat[:3]).R  # spatialmath wants [w, x, y, z]
        T_target = SE3.Rt(R_mat, target_pos)


        q0 = np.concatenate(([rail_seed], q_seed_arm))
        sol = self.robot.ikine_LM(T_target, end=EE_LINK, q0=q0,
                                    mask=[1, 1, 1, 1, 1, 1], tol=1e-4)
        return sol.q[0],sol.q[1:], sol



    def compute_jacobian(self, q_arm):
        return self.robot.jacobe(q_arm, end=EE_LINK, start='base_link')

    def _failure_reason(self, sol, res, condition_number_threshold,
                         max_joint_vel_threshold, rail_fallback_exhausted):
        suffix = ' (rail-assisted recovery also exhausted)' if rail_fallback_exhausted else ''
        if res is None:
            return f'IK did not converge (reason={sol.reason}){suffix}'
        if res['low_cond']:
            return f"Singularity: condition number {res['cond_num']:.2f} > {condition_number_threshold}{suffix}"
        if res['jump']:
            bad_arm = np.where(res['joint_vel'][1:] > max_joint_vel_threshold)[0].tolist()
            rail_bad = res['joint_vel'][0] > max_rail_vel_threshold
            return (f"Joint/Rail velocity exceeded limits: Rail exceeded={rail_bad} "
            f"(vel={res['joint_vel'][0]:.3f} m/s), Arm indices={bad_arm}{suffix}")
        if res['collide']:
            return f"Collision: q={np.round(res['q_full'], 3)}{suffix}"
        return f'Unknown failure{suffix}'

    def _solve_waypoint_with_recovery(self, target_pos, target_quat, seed_arm, rail_pos, prev_rail=None,
                                       prev_arm=None, check_jump=False,
                                       dt_waypoint=None,
                                       max_rail_vel_threshold=2.0,
                                       max_joint_vel_threshold=2.0,
                                       condition_number_threshold=50.0,
                                       max_attempts=10,
                                       max_rail_attempts=10,
                                       verbose=False, label=''):
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
        def evaluate(q_arm, sol, rail):
            if not sol.success:
                return None
            q_full = np.concatenate(([rail], q_arm))
            J = self.compute_jacobian(q_arm)
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
                    rail_jump = rail_vel > max_rail_vel_threshold
                else:
                    rail_vel = 0.0
                    rail_jump = False

                jump = arm_jump or rail_jump
                
                # Concatenate joint velocities for reporting: [rail_vel, arm_vel...]
                joint_vel = np.concatenate(([rail_vel if prev_rail is not None else 0.0], arm_vel))
            else:
                joint_vel = np.zeros(7)
                jump = False

            if len(q_full) != 7:
                raise ValueError(f"Expected q_full length 7, got {len(q_full)}")
                
            collide = self.check_all_collisions(q_full, verbose=verbose)
            return dict(q_full=q_full, cond_num=cond_num, low_cond=low_cond,
                        jump=jump, collide=collide, joint_vel=joint_vel)

        # True 7DOF rail solve
        search_radius = 0.05
        for attempt in range(max_rail_attempts + 1):
            this_seed = seed_arm if attempt == 0 else (
                seed_arm + (2 * np.random.rand(6) - 1) * search_radius)
            new_rail, q_arm, sol = self.solve_ik_lm(target_pos, target_quat, this_seed, rail_seed=rail_pos)
            res = evaluate(q_arm, sol, new_rail)
            if res is not None and not (res['low_cond'] or res['jump'] or res['collide']):
                if verbose:
                    print(f'{label}rail-assisted recovery succeeded '
                          f'(rail {rail_pos:.3f} -> {new_rail:.3f})')
                return dict(ok=True, q_arm=q_arm, q_full=res['q_full'], rail_pos=new_rail,
                            cond_num=res['cond_num'], joint_vel=res['joint_vel'],
                            attempts_used=max_attempts + attempt, rail_moved=True, reason=None)
            last = (q_arm, sol, res)
            search_radius += 0.05

        q_arm, sol, res = last
        reason = self._failure_reason(sol, res, condition_number_threshold,max_rail_vel_threshold,
                                       max_joint_vel_threshold, rail_fallback_exhausted=True)
        return dict(ok=False, q_arm=q_arm, q_full=(res['q_full'] if res else None),
                    rail_pos=rail_pos, cond_num=(res['cond_num'] if res else None),
                    joint_vel=(res['joint_vel'] if res else None),
                    attempts_used=max_attempts + max_rail_attempts, rail_moved=False, reason=reason)

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

    def find_feasible_segments(self, ee_x, ee_y, ee_z, ee_quat, q_seed, min_length,
                                    dt_waypoint,
                                    max_rail_vel_threshold=2.0,
                                    max_joint_vel_threshold=2.0,
                                    condition_number_threshold=50.0,
                                    max_attempts=10,
                                    verbose=False):
            num_pts = len(ee_x)
            pos = np.column_stack((ee_x, ee_y, ee_z))
            quat = np.asarray(ee_quat)
            rail_pos = q_seed[0]  # running value -- may move if rail fallback ever fires
            q_home_arm = q_seed[1:]

            def try_point(target_pos, target_quat, seed_arm, prev_rail, prev_arm, check_jump):
                result = self._solve_waypoint_with_recovery(
                    target_pos, target_quat, np.asarray(seed_arm)[-6:], rail_pos, prev_rail=prev_rail,
                    prev_arm=(np.asarray(prev_arm)[-6:] if prev_arm is not None else None),
                    check_jump=check_jump, dt_waypoint=dt_waypoint,
                    max_rail_vel_threshold=max_rail_vel_threshold,
                    max_joint_vel_threshold=max_joint_vel_threshold,
                    condition_number_threshold=condition_number_threshold,
                    max_attempts=max_attempts, 
                    verbose=verbose, label='[segment scan] ')
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
                    ok, q_arm, q_full, rail = try_point(target_pos, target_quat, q_home_arm, None,None, check_jump=False)
                    if ok:
                        current_start = idx
                        current_qs = [q_full]
                        prev_config_arm = q_arm
                        rail_prev = rail  # persists if this entry solve needed the rail fallback
                    elif verbose:

                        print(f'[segment scan] point {idx}: infeasible even as a fresh entry, skipping')
                else:
                    ok, q_arm, q_full, rail = try_point(target_pos, target_quat, prev_config_arm,rail_prev, prev_config_arm, check_jump=True)
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

    def process_feasible_segment(self, segment, q_start, dt_waypoint,
                                  t_transition=2.0, verbose=False):
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

        t_waypoints_full = np.concatenate(([0], np.linspace(t_transition, t_final, seg_len)))
        t_sim = np.linspace(0, t_final, num_frames)

        pchip_interpolator = PchipInterpolator(t_waypoints_full, config_soln, axis=0)
        q_interp = pchip_interpolator(t_sim)

        dt_sim = 1.0 / self.framerate
        q_dot = np.gradient(q_interp, dt_sim, axis=0)

        if verbose:
            print(f'[segment interp] segment [{segment["start_idx"]}, {segment["end_idx"]}] '
                  f'(len={seg_len}) -> {num_frames} frames over {t_final:.2f}s')

        return q_dot, q_interp

    def process_matlab_validation(self, ee_x, ee_y, ee_z, ee_quat, q_start,
                                  max_rail_vel_threshold=2.0,
                                   max_joint_vel_threshold=2.0,
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
        assert dt_waypoint==1, f'dt_waypoint={dt_waypoint} differs from SISFOS Default 0.1'
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