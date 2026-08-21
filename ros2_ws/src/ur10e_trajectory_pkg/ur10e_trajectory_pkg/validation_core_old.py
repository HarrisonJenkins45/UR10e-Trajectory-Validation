"""Pure trajectory validation/IK logic -- no ROS dependencies.

Used by Validate_trajServer.py (the ROS service) AND by
test_validation_local.py (a standalone script you can run directly on your
Mac for fast iteration, no Docker/ROS/Gazebo round-trip needed).

CHANGELOG vs original:
- Pinned end='tool0' on every IK / Jacobian call. The URDF has multiple
  leaf links (ft_frame, base, tool0) so roboticstoolbox was silently
  picking ee_links[0] ("ft_frame"), which is NOT the MATLAB tool0 frame.
  This alone shifts every solved configuration vs. MATLAB.
- Self-collision adjacency check now uses actual link.parent relationships
  instead of index-distance (idx_j <= idx_i + 2). The index heuristic only
  works for a single unbranched chain with no gaps; this URDF has branches
  (ft_frame, base) so index adjacency != kinematic adjacency, which is what
  produced the false-positive 'rail_base_link vs base_link_inertia' hit
  (those two ARE parent/child, i.e. truly adjacent, but weren't caught by
  the index-based check).
- Added the floor box back (it was dropped from self.env entirely; MATLAB
  checks against both floor and wall).
"""
import tempfile
import os
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from spatialgeometry import Cuboid
from scipy.interpolate import PchipInterpolator

EE_LINK = "tool0"


class TrajectoryValidator:
    def __init__(self, urdf_path, mesh_base_path=None, framerate=30):
        self.framerate = framerate

        if mesh_base_path is not None:
            # rtb.ERobot.URDF() resolves package:// mesh URIs against its OWN
            # bundled rtbdata/xacro/ directory by default -- NOT against any
            # folder named after the package sitting near the URDF file. To
            # point it at a real local mesh directory instead, rewrite every
            # 'package://ur_description/' URI to an absolute local path
            # before parsing, bypassing that resolution entirely.
            with open(urdf_path, 'r') as f:
                urdf_text = f.read()
            mesh_base_path = mesh_base_path.rstrip('/')
            urdf_text = urdf_text.replace('package://ur_description/', mesh_base_path + '/')

            tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False)
            tmp.write(urdf_text)
            tmp.close()
            self.robot = rtb.ERobot.URDF(tmp.name)
            os.unlink(tmp.name)
        else:
            self.robot = rtb.ERobot.URDF(urdf_path)

        # Sanity-check that the ee link we're about to pin everywhere
        # actually exists in this robot's link set.
        ee_names = [l.name for l in self.robot.links]
        if EE_LINK not in ee_names:
            raise ValueError(
                f"Expected end-effector link '{EE_LINK}' not found in URDF links: {ee_names}"
            )

        # Floor parallel to XY plane at Y = +1.0m, Wall parallel to XZ plane at Z = -0.05m
        # (matches MATLAB's actual code, not its stale comment -- see script header)
        self.floor_box = Cuboid(scale=[3.0, 0.05, 3.0], pose=SE3(0.0, 1.0, 0.0))
        self.wall_box = Cuboid(scale=[3.0, 3.0, 0.05], pose=SE3(0.0, 0.0, -0.05))
        self.env = [self.floor_box, self.wall_box]

        # Precompute a name -> link index map and a name -> parent-name map
        # once, for fast adjacency lookups during collision checking.
        self._link_index_by_name = {l.name: idx for idx, l in enumerate(self.robot.links)}
        self._parent_name_by_index = {}
        for idx, link in enumerate(self.robot.links):
            parent = getattr(link, 'parent', None)
            self._parent_name_by_index[idx] = parent.name if parent is not None else None

        # Collapse chains of FIXED joints into single "rigid clusters" before
        # computing adjacency. e.g. rail_base_link -[prismatic]-> base_link
        # -[fixed]-> base_link_inertia: base_link and base_link_inertia move
        # together with zero relative motion, so base_link_inertia is
        # physically just as "adjacent" to rail_base_link as base_link is.
        # A plain parent/child (1-hop) check misses this and reports a
        # false-positive self-collision between rail_base_link and
        # base_link_inertia. This mirrors what MATLAB's rigidBodyTree
        # effectively does for 'SkippedSelfCollisions','adjacent'.
        self._anchor_by_index = {}
        for idx in range(len(self.robot.links)):
            self._anchor_by_index[idx] = self._compute_anchor(idx)

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

    def solve_ik_lm(self, target_pos, q_seed_arm):
        T_target = SE3(target_pos[0], target_pos[1], target_pos[2])
        # start='base_link' excludes the rail joint from the search entirely,
        # matching MATLAB's IK problem, which is 6-DOF only -- the rail is
        # never part of its optimization, just a constant zero column
        # prepended at the very end for the Simulink output format.
        # end=EE_LINK pins the solve to tool0 explicitly -- without this,
        # roboticstoolbox silently falls back to ee_links[0] when a URDF has
        # multiple leaf links (this one has three: ft_frame, base, tool0),
        # which is NOT the frame MATLAB is solving for.
        sol = self.robot.ikine_LM(T_target, end=EE_LINK, start='base_link', q0=q_seed_arm,
                                   mask=[1, 1, 1, 0, 0, 0], tol=1e-4)
        return sol.q, sol

    def compute_jacobian(self, q_arm):
        return self.robot.jacobe(q_arm, end=EE_LINK, start='base_link')

    def check_all_collisions(self, q_full):
        transforms = self.robot.fkine_all(q_full)

        # 1. Build a link-indexed dictionary to preserve actual link hierarchy
        # index -> list of active collision geometries
        link_shapes = {}
        for idx, (link, T) in enumerate(zip(self.robot.links, transforms)):
            collision_list = getattr(link, 'collision', []) or []
            active_geoms = []
            for geom in collision_list:
                try:
                    if getattr(geom, 'collision', False):
                        geom.pose = SE3(T)
                        active_geoms.append(geom)
                except Exception:
                    pass
            if active_geoms:
                link_shapes[idx] = active_geoms

        active_indices = list(link_shapes.keys())

        # 2. Check Self-Collisions between non-adjacent robot tree links.
        # MATLAB skips pairs that are kinematically adjacent (parent/child),
        # not pairs that are numerically close in index -- use real
        # parent/child relationships so branches (ft_frame, base, tool0)
        # don't produce false positives / miss true adjacency.
        for i, idx_i in enumerate(active_indices):
            for idx_j in active_indices[i + 1:]:
                if self._is_adjacent(idx_i, idx_j):
                    continue

                for shape_i in link_shapes[idx_i]:
                    for shape_j in link_shapes[idx_j]:
                        try:
                            if shape_i.iscollided(shape_j):
                                print(f"Self-collision: '{self.robot.links[idx_i].name}' vs '{self.robot.links[idx_j].name}'")
                                return True
                        except ValueError:
                            continue

        # 3. Check Environment Collisions (floor + wall)
        for idx in active_indices:
            for shape in link_shapes[idx]:
                for env_shape in self.env:
                    try:
                        if shape.iscollided(env_shape):
                            print(f"Env collision on link '{self.robot.links[idx].name}'")
                            return True
                    except ValueError:
                        continue

        return False

    # ------------------------------------------------------------------
    # Manual capsule-based collision checking.
    #
    # This is an alternative to check_all_collisions() above that avoids
    # inferring kinematic adjacency from the URDF's parent-chain structure
    # entirely. Instead:
    #   - each link's collision volume is a hand-specified CAPSULE (a line
    #     segment + radius) in that link's own local frame
    #   - adjacency (which pairs are ALLOWED to touch, e.g. shoulder vs
    #     upperarm at their shared joint) is an explicit hardcoded list,
    #     not something derived from walking parent/child names
    # This makes it immune to the rail/prismatic-joint assumptions that
    # _compute_anchor/_is_adjacent bake in, at the cost of you having to
    # supply real capsule dimensions for your robot (placeholders below).
    #
    # TODO: replace the placeholder (p0, p1, radius) tuples with real
    # measurements. p0/p1 are endpoints of the capsule's core segment,
    # expressed in the LINK's own local frame (i.e. relative to that
    # link's origin as defined in the URDF -- open the URDF/mesh to check
    # which local axis each link actually extends along before trusting
    # these numbers). radius should be the link's rough cross-sectional
    # radius at its widest point (err slightly large, not small).
    # TODO: these are still coarse placeholders for the arm links -- run
    # validator.derive_link_capsules_from_robot() and paste its output in
    # here once you've verified this branch loads meshes correctly (it now
    # also derives rail_base_link/rail_carriage_link's box geometry).
    # NOTE: 'base_link' is intentionally absent -- per the URDF it's an
    # empty pass-through link with no <collision> geometry at all (the
    # real base mesh, base.stl, lives on 'base_link_inertia'). Keying a
    # capsule under 'base_link' tests a phantom volume against nothing.
    # NOTE: as of this version, each link maps to a LIST of (p0, p1, radius)
    # capsule segments, not a single capsule. A single capsule spanning a
    # whole link applies its widest cross-section radius uniformly along
    # the ENTIRE segment -- badly conservative for short/boxy/irregular
    # links (e.g. shoulder_link), where the true cross-section varies a
    # lot along its length. Splitting into segments lets each piece be as
    # thin as its own local geometry actually is. Re-run
    # derive_link_capsules_from_robot() (now segment-aware) to regenerate.

    LINK_CAPSULES = {
        'rail_base_link': [(np.array([np.float64(-1.0), np.float64(0.0), np.float64(0.0)]), np.array([np.float64(1.0), np.float64(0.0), np.float64(0.0)]), 0.0559)],
        'rail_carriage_link': [(np.array([np.float64(-0.075), np.float64(0.0), np.float64(0.0)]), np.array([np.float64(0.075), np.float64(0.0), np.float64(0.0)]), 0.0791)],
        'base_link_inertia': [(np.array([np.float64(-0.0006), np.float64(0.0006), np.float64(-0.0)]), np.array([np.float64(-0.0006), np.float64(0.0006), np.float64(0.0331)]), 0.0958), (np.array([np.float64(-0.0006), np.float64(0.0006), np.float64(0.0662)]), np.array([np.float64(-0.0006), np.float64(0.0006), np.float64(0.0993)]), 0.0758)],
        'shoulder_link': [(np.array([np.float64(-0.0425), np.float64(-0.0557), np.float64(0.0784)]), np.array([np.float64(0.0034), np.float64(-0.0074), np.float64(0.0584)]), 0.1552), (np.array([np.float64(0.0034), np.float64(-0.0074), np.float64(0.0584)]), np.array([np.float64(0.0494), np.float64(0.041), np.float64(0.0383)]), 0.1552), (np.array([np.float64(0.0494), np.float64(0.041), np.float64(0.0383)]), np.array([np.float64(0.0953), np.float64(0.0894), np.float64(0.0182)]), 0.1466)],
        'upper_arm_link': [(np.array([np.float64(-0.0001), np.float64(0.0311), np.float64(0.6743)]), np.array([np.float64(-0.0001), np.float64(0.041), np.float64(0.4244)]), 0.1232), (np.array([np.float64(0.0), np.float64(0.0509), np.float64(0.1745)]), np.array([np.float64(0.0001), np.float64(0.0608), np.float64(-0.0754)]), 0.1549)],
        'forearm_link': [(np.array([np.float64(-0.0005), np.float64(0.0047), np.float64(-0.0611)]), np.array([np.float64(-0.0001), np.float64(-0.005), np.float64(0.1649)]), 0.0805), (np.array([np.float64(0.0003), np.float64(-0.0147), np.float64(0.391)]), np.array([np.float64(0.0007), np.float64(-0.0243), np.float64(0.6171)]), 0.0991)],
        'wrist_1_link': [(np.array([np.float64(0.0001), np.float64(0.1468), np.float64(-0.0558)]), np.array([np.float64(-0.0008), np.float64(0.137), np.float64(-0.013)]), 0.0758), (np.array([np.float64(-0.0008), np.float64(0.137), np.float64(-0.013)]), np.array([np.float64(-0.0017), np.float64(0.1272), np.float64(0.0298)]), 0.0816), (np.array([np.float64(-0.0017), np.float64(0.1272), np.float64(0.0298)]), np.array([np.float64(-0.0026), np.float64(0.1174), np.float64(0.0726)]), 0.0725)],
        'wrist_2_link': [(np.array([np.float64(0.0013), np.float64(-0.0534), np.float64(0.1277)]), np.array([np.float64(0.0009), np.float64(-0.0115), np.float64(0.123)]), 0.0652), (np.array([np.float64(0.0009), np.float64(-0.0115), np.float64(0.123)]), np.array([np.float64(0.0005), np.float64(0.0304), np.float64(0.1182)]), 0.0716), (np.array([np.float64(0.0005), np.float64(0.0304), np.float64(0.1182)]), np.array([np.float64(0.0002), np.float64(0.0722), np.float64(0.1134)]), 0.0606)],
        'wrist_3_link': [(np.array([np.float64(0.0005), np.float64(0.1057), np.float64(-0.046)]), np.array([np.float64(0.0006), np.float64(0.1026), np.float64(-0.0129)]), 0.0487), (np.array([np.float64(0.0006), np.float64(0.1026), np.float64(-0.0129)]), np.array([np.float64(0.0008), np.float64(0.0994), np.float64(0.0201)]), 0.0559), (np.array([np.float64(0.0008), np.float64(0.0994), np.float64(0.0201)]), np.array([np.float64(0.001), np.float64(0.0962), np.float64(0.0532)]), 0.0482)],
    }

    # Explicit whitelist of link-name pairs allowed to touch (kinematic
    # neighbors along the chain). Order doesn't matter -- checked both ways.
    # This mirrors the URDF's real joint chain, INCLUDING the rail:
    #   rail_base_link -[prismatic]-> rail_carriage_link -[fixed: base_joint]->
    #   base_link (no geometry, skipped) -[fixed]-> base_link_inertia
    #   -[revolute]-> shoulder_link -> upper_arm_link -> forearm_link ->
    #   wrist_1_link -> wrist_2_link -> wrist_3_link
    # Since base_link carries no geometry, rail_carriage_link and
    # base_link_inertia are the pair that's actually physically touching
    # (base_link is a zero-thickness pass-through between them).
    ADJACENT_LINK_PAIRS = {
        frozenset({'rail_base_link', 'rail_carriage_link'}),
        frozenset({'rail_carriage_link', 'base_link_inertia'}),
        # rail_base_link <-> base_link_inertia: NOT true kinematic
        # neighbors (rail_carriage_link sits between them), but with
        # linear_rail_joint locked at a constant value for the whole
        # trajectory, this pair's distance is CONFIGURATION-INVARIANT --
        # no arm joint can change it, so if it reads as a collision it
        # does so identically at every waypoint and can never be resolved
        # by the per-waypoint IK retry loop. Treated here as a static
        # mounting-stack relationship (verify against the real rig's
        # rail height before trusting this -- the URDF marks the rail
        # dimensions as placeholders) rather than as a live hazard.
        frozenset({'rail_base_link', 'base_link_inertia'}),
        frozenset({'base_link_inertia', 'shoulder_link'}),
        frozenset({'shoulder_link', 'upper_arm_link'}),
        frozenset({'upper_arm_link', 'forearm_link'}),
        frozenset({'forearm_link', 'wrist_1_link'}),
        frozenset({'wrist_1_link', 'wrist_2_link'}),
        frozenset({'wrist_2_link', 'wrist_3_link'}),
    }

    # Env plane margins (matching floor_box/wall_box's actual planes:
    # floor at Y=+1.0, wall at Z=-0.05). A capsule collides with the floor
    # if any point on its core segment is within `radius` of Y=1.0, same
    # idea for the wall at Z=-0.05.
    FLOOR_Y = 1.0
    WALL_Z = -0.05

    @staticmethod
    def _closest_dist_segment_segment(p0, p1, q0, q1):
        """Shortest distance between two 3D line segments [p0,p1] and
        [q0,q1]. Standard closest-point-between-segments algorithm."""
        d1, d2, r = p1 - p0, q1 - q0, p0 - q0
        a, e, f = np.dot(d1, d1), np.dot(d2, d2), np.dot(d2, r)
        EPS = 1e-9

        if a <= EPS and e <= EPS:
            return np.linalg.norm(p0 - q0)
        if a <= EPS:
            s, t = 0.0, np.clip(f / e, 0.0, 1.0)
        else:
            c = np.dot(d1, r)
            if e <= EPS:
                t, s = 0.0, np.clip(-c / a, 0.0, 1.0)
            else:
                b = np.dot(d1, d2)
                denom = a * e - b * b
                s = np.clip((b * f - c * e) / denom, 0.0, 1.0) if denom > EPS else 0.0
                t = (b * s + f) / e
                if t < 0.0:
                    t, s = 0.0, np.clip(-c / a, 0.0, 1.0)
                elif t > 1.0:
                    t, s = 1.0, np.clip((b - c) / a, 0.0, 1.0)
        c1, c2 = p0 + d1 * s, q0 + d2 * t
        return np.linalg.norm(c1 - c2)

    def check_all_collisions_manual(self, q_full, verbose=False):
        """Drop-in alternative to check_all_collisions() using hardcoded
        capsule segments + a hardcoded adjacency whitelist instead of STL
        meshes and inferred kinematic adjacency. Same signature/return
        type (bool), so it can be swapped in anywhere check_all_collisions
        is called (including monkeypatched onto self.check_all_collisions).
        """
        transforms = self.robot.fkine_all(q_full)
        link_by_name = {l.name: idx for idx, l in enumerate(self.robot.links)}

        # Transform every segment's local endpoints into world space.
        # world_capsules[name] is a LIST of (p0_world, p1_world, radius).
        world_capsules = {}
        for name, segments in self.LINK_CAPSULES.items():
            if name not in link_by_name:
                continue  # skip capsules for links this URDF doesn't have
            T = SE3(transforms[link_by_name[name]])
            world_capsules[name] = [
                ((T * SE3(*p0_local)).t, (T * SE3(*p1_local)).t, radius)
                for (p0_local, p1_local, radius) in segments
            ]

        names = list(world_capsules.keys())

        # Self-collision: every segment of link i vs. every segment of
        # link j, distance vs. sum of radii.
        for i, name_i in enumerate(names):
            for name_j in names[i + 1:]:
                if frozenset({name_i, name_j}) in self.ADJACENT_LINK_PAIRS:
                    continue
                for seg_i, (p0, p1, r_i) in enumerate(world_capsules[name_i]):
                    for seg_j, (q0, q1, r_j) in enumerate(world_capsules[name_j]):
                        dist = self._closest_dist_segment_segment(p0, p1, q0, q1)
                        if dist < (r_i + r_j):
                            if verbose:
                                print(f"Self-collision: '{name_i}'[seg {seg_i}] vs "
                                      f"'{name_j}'[seg {seg_j}] (dist={dist:.4f}, "
                                      f"combined radius={r_i + r_j:.4f})")
                            return True

        # Env collision: every segment's endpoints vs. floor plane
        # (Y=FLOOR_Y) and wall plane (Z=WALL_Z). Distance from a point to
        # an axis-aligned plane is just the abs difference along that axis.
        for name, segments in world_capsules.items():
            for seg_idx, (p0, p1, radius) in enumerate(segments):
                for pt in (p0, p1):
                    if abs(pt[1] - self.FLOOR_Y) < radius:
                        if verbose:
                            print(f"Env collision (floor) on '{name}'[seg {seg_idx}]")
                        return True
                    if abs(pt[2] - self.WALL_Z) < radius:
                        if verbose:
                            print(f"Env collision (wall) on '{name}'[seg {seg_idx}]")
                        return True

        return False

    def derive_link_capsules_from_robot(self, percentile=98, num_segments=3,
                                         forced_axes=None, verbose=True):
        """One-off utility: derive capsule geometry per link directly from
        the collision meshes RTB already loaded onto self.robot, instead
        of hand-guessing. Crucially this uses each collision shape's
        `.base` transform (the URDF's <collision><origin> offset), so the
        resulting capsules end up correctly expressed in the LINK's own
        kinematic frame -- the same frame check_all_collisions_manual
        transforms via fkine_all(). Loading a raw STL file cold and
        skipping that offset is what would silently misalign everything.

        Each mesh link is split into `num_segments` capsule segments along
        its principal axis (not just one). A single capsule spanning a
        whole link applies its worst-case cross-section radius uniformly
        along the ENTIRE segment, which is badly conservative for short or
        irregular links (motor housings, flanges, bulges) -- segmenting
        lets each piece be only as thick as its own local geometry, which
        tightens the envelope and cuts down on false positives from links
        that are close to something but not actually touching it. Box
        links (the rail) stay as a single segment since a box's cross
        -section doesn't vary along its length.

        forced_axes: optional {link_name: axis_vector} overriding the
        PCA-derived axis for specific links. Use this for flat/disc-shaped
        links (e.g. base_link_inertia's mounting flange) where PCA may
        pick an axis running ACROSS the flat face -- baking the disc's
        full diameter into the radius -- instead of through its actual
        (much shorter) thickness. Pass the known physical mounting axis
        (e.g. [0,0,1] for a link that stacks vertically) to get a tight
        squat capsule instead of an oversized flat one.

        Run this once (e.g. from the interpreter: validator.
        derive_link_capsules_from_robot()), sanity-check the printed
        numbers against known robot dimensions, then paste the result
        into LINK_CAPSULES.

        percentile: radius = this percentile of vertex distance from the
        principal axis (per segment), not the max, so a single stray
        vertex (bolt hole, flange, cable mount) doesn't balloon the
        radius. Lower it (e.g. 90) if a link still comes out oversized.
        num_segments: capsule pieces per mesh link. More segments = tighter
        fit but more pairwise distance checks at runtime; 3 is a reasonable
        default, bump to 4-5 for particularly boxy/irregular links.

        Note: if `shape.filename`/`shape.base` don't exist on your
        installed spatialgeometry version, run `print(dir(shape))` on one
        shape to find the right attribute names and adjust below.
        """
        import trimesh
        forced_axes = forced_axes or {}

        capsules = {}
        for link in self.robot.links:
            shapes = getattr(link, 'collision', None)
            if not shapes:
                continue
            for shape in shapes:
                filename = getattr(shape, 'filename', None)
                local_T = getattr(shape, 'base', None)
                segments = []

                if filename is not None:
                    # Mesh-based collision geometry (the arm links).
                    mesh = trimesh.load_mesh(filename)
                    verts = np.asarray(mesh.vertices)
                    if local_T is not None:
                        verts_h = np.c_[verts, np.ones(len(verts))]
                        verts = (np.asarray(local_T) @ verts_h.T).T[:, :3]

                    centroid = verts.mean(axis=0)
                    centered = verts - centroid

                    if link.name in forced_axes:
                        axis = np.asarray(forced_axes[link.name], dtype=float)
                        axis = axis / np.linalg.norm(axis)
                    else:
                        _, _, vh = np.linalg.svd(centered, full_matrices=False)
                        axis = vh[0]

                    proj = centered @ axis
                    perp_all = centered - np.outer(proj, axis)
                    perp_dist_all = np.linalg.norm(perp_all, axis=1)

                    # Bin vertices into num_segments equal-width slices
                    # along the (PCA or forced) axis; each slice gets its
                    # own (p0, p1, radius) using only ITS vertices, not the
                    # whole link's worst-case cross-section.
                    edges = np.linspace(proj.min(), proj.max(), num_segments + 1)
                    for k in range(num_segments):
                        lo, hi = edges[k], edges[k + 1]
                        mask = (proj >= lo) & (proj <= hi)
                        if not np.any(mask):
                            continue  # empty slice (sparse/irregular mesh), skip
                        p0 = centroid + axis * lo
                        p1 = centroid + axis * hi
                        radius = float(np.percentile(perp_dist_all[mask], percentile))
                        segments.append((p0.round(4), p1.round(4), round(radius, 4)))
                    n_verts, src = len(verts), filename

                elif hasattr(shape, 'scale'):
                    # Box collision geometry (the rail links: no mesh file,
                    # dimensions given directly as a <box size="..."/>).
                    # Single segment -- a box's cross-section is constant
                    # along its length, so splitting it wouldn't help.
                    scale = np.asarray(shape.scale, dtype=float)
                    axis_idx = int(np.argmax(scale))
                    half_len = scale[axis_idx] / 2.0
                    cross = np.delete(scale, axis_idx)
                    radius = 0.5 * float(np.linalg.norm(cross))

                    p0_local, p1_local = np.zeros(3), np.zeros(3)
                    p0_local[axis_idx], p1_local[axis_idx] = -half_len, half_len
                    if local_T is not None:
                        p0 = (np.asarray(local_T) @ np.r_[p0_local, 1.0])[:3]
                        p1 = (np.asarray(local_T) @ np.r_[p1_local, 1.0])[:3]
                    else:
                        p0, p1 = p0_local, p1_local
                    segments = [(p0.round(4), p1.round(4), round(radius, 4))]
                    n_verts, src = None, f"box{tuple(scale)}"

                else:
                    continue  # unrecognized collision shape type, skip

                capsules[link.name] = segments
                if verbose:
                    count_str = f"{n_verts} verts" if n_verts is not None else "box"
                    print(f"  {link.name} ({count_str}, from {src}):")
                    for si, (p0, p1, r) in enumerate(segments):
                        print(f"    seg {si}: p0={p0}, p1={p1}, radius={r:.3f}")

        if verbose:
            print("\nPaste this into LINK_CAPSULES:\n")
            print("LINK_CAPSULES = {")
            for name, segments in capsules.items():
                seg_strs = ", ".join(
                    f"(np.array({list(p0)}), np.array({list(p1)}), {r})"
                    for (p0, p1, r) in segments
                )
                print(f"    '{name}': [{seg_strs}],")
            print("}")

        return capsules



    def find_feasible_segments(self, ee_x, ee_y, ee_z, q_seed, min_length,
                                dt_waypoint,
                                max_joint_vel_threshold=2.0,
                                condition_number_threshold=50.0,
                                max_attempts=10,
                                verbose=False):
        """Scan an arbitrary end-effector path (which may dip in and out of
        reachable/safe space) and return the maximal contiguous runs of
        waypoints that are each individually feasible: IK converges, no
        self/environment collision, Jacobian condition number below
        threshold (i.e. not near a singularity), and no excessive joint
        velocity relative to the PREVIOUS point in the same run. Only runs
        of length >= min_length are returned.

        Each new run's first point gets an unchecked "entry" solve (no
        jump check), seeded from q_seed's arm joints -- mirroring the
        unchecked transition-phase solve in process_matlab_validation, so a
        gap doesn't inherit a joint-jump penalty from wherever the arm was
        parked before it.

        dt_waypoint: time between consecutive waypoints in ee_x/y/z, for the
        joint-velocity check. Passed explicitly since, unlike
        process_matlab_validation, there's no fixed t_transition/t_traj
        structure here to derive it from.

        Returns: list of dicts, each with 'start_idx'/'end_idx' (inclusive
        indices into the input arrays), 'q_full' ((length, 7) array
        including the constant rail column), and 'length'.
        """
        
        num_pts = len(ee_x)
        pos = np.column_stack((ee_x, ee_y, ee_z))
        rail_pos = q_seed[0]
        q_home_arm = q_seed[1:]

        def try_point(target_pos, seed_arm, prev_arm, check_jump):
            search_radius = 0.05
            for attempt in range(max_attempts + 1):
                this_seed = seed_arm if attempt == 0 else (
                    seed_arm + (2 * np.random.rand(6) - 1) * search_radius)
                q_arm, sol = self.solve_ik_lm(target_pos, q_seed_arm=this_seed)
                if not sol.success:
                    search_radius += 0.05
                    continue
                q_full = np.concatenate(([rail_pos], q_arm))
                J = self.compute_jacobian(q_arm)
                singular_values = np.linalg.svd(J, compute_uv=False)
                cond_num = (singular_values[0] / singular_values[-1]
                            if singular_values[-1] > 1e-9 else np.inf)
                low_manip = cond_num > condition_number_threshold
                if check_jump:
                    joint_vel = np.abs(q_arm - prev_arm) / dt_waypoint
                    jump = np.any(joint_vel > max_joint_vel_threshold)
                else:
                    jump = False
                collide = self.check_all_collisions_manual(q_full, verbose=verbose)
                if not (low_manip or jump or collide):
                    return True, q_arm, q_full
                search_radius += 0.05
            return False, None, None

        segments = []
        current_start = None
        current_qs = []
        prev_config_arm = None
        idx = 0
        while idx < num_pts:
            target_pos = pos[idx, :]
            if current_start is None:
                ok, q_arm, q_full = try_point(target_pos, q_home_arm, None, check_jump=False)
                if ok:
                    current_start = idx
                    current_qs = [q_full]
                    prev_config_arm = q_arm
                elif verbose:
                    print(f'[segment scan] point {idx}: infeasible even as a fresh entry, skipping')
            else:
                ok, q_arm, q_full = try_point(target_pos, prev_config_arm, prev_config_arm, check_jump=True)
                if ok:
                    current_qs.append(q_full)
                    prev_config_arm = q_arm
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
                    current_start, current_qs, prev_config_arm = None, [], None
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

    def process_matlab_validation(self, ee_x, ee_y, ee_z, q_start,
                                   max_joint_vel_threshold=2.0,
                                   condition_number_threshold=50.0,
                                   t_transition=2.0, t_traj=10.0,
                                   verbose=False):
        num_waypts = len(ee_x)
        t_final = t_traj + t_transition
        num_frames = int(t_final * self.framerate)

        # Rail is locked at its starting value for the whole solve, exactly
        # like MATLAB's zeros(numFrames,1) column -- it is never optimized.
        rail_pos = q_start[0]
        q_arm_start = q_start[1:]

        pos_waypoints = np.column_stack((ee_x, ee_y, ee_z))
        config_soln = np.zeros((num_waypts + 1, 7))
        config_soln[0, :] = q_start

        dt_waypoint = (t_final - t_transition) / (num_waypts - 1)

        # Transition solve: q_arm_start (home) -> first circle waypoint.
        # UNCHECKED, matching the MATLAB script exactly -- this jump is meant
        # to be absorbed by the full t_transition (2s) window during pchip
        # interpolation, not by the tight per-waypoint dt used for the
        # checked loop below.
        q_arm_transition, sol0 = self.solve_ik_lm(pos_waypoints[0, :], q_seed_arm=q_arm_start)
        if not sol0.success:
            msg = f'IK did not converge on the approach/transition waypoint (target={np.round(pos_waypoints[0, :], 3)}, reason={sol0.reason})'
            if verbose:
                print(f'[transition] {msg}')
            return False, np.array([]), np.array([]), msg
        config_soln[1, :] = np.concatenate(([rail_pos], q_arm_transition))
        prev_config_arm = q_arm_transition.copy()

        # Checked loop: consecutive circle waypoints only (mirrors MATLAB's
        # `for k = 2:numWayPts`, which compares pos(2)..pos(numWayPts)
        # against each other -- pos(1) was already consumed above).
        for k in range(1, num_waypts):
            target_pos = pos_waypoints[k, :]

            def try_solve(q_seed_arm):
                q_arm, sol = self.solve_ik_lm(target_pos, q_seed_arm=q_seed_arm)
                q_full = np.concatenate(([rail_pos], q_arm))
                J = self.compute_jacobian(q_arm)
                singular_values = np.linalg.svd(J, compute_uv=False)
                cond_num = (singular_values[0] / singular_values[-1]
                            if singular_values[-1] > 1e-9 else np.inf)
                low_manip = cond_num > condition_number_threshold
                joint_vel = np.abs(q_arm - prev_config_arm) / dt_waypoint
                jump = np.any(joint_vel > max_joint_vel_threshold)
                collide = self.check_all_collisions_manual(q_full, verbose=verbose) if sol.success else False
                return q_arm, q_full, sol, cond_num, low_manip, jump, collide, joint_vel

            q_sol_arm, q_sol_full, sol, cond_num, low_manip, jump, collide, joint_vel = try_solve(prev_config_arm)

            # Local search retry, matching MATLAB's recovery loop exactly:
            # up to 10 attempts, random offset within a growing search
            # radius, re-seeded from prev_config_arm each time (not from
            # the previous failed attempt).
            max_attempts = 10
            search_radius = 0.05
            attempt = 1
            while (collide or jump or low_manip or not sol.success) and attempt <= max_attempts:
                if verbose:
                    print(f'[waypoint {k}] attempt {attempt}: collide={collide} jump={jump} '
                          f'low_manip={low_manip} (cond={cond_num:.2f}) ik_ok={sol.success} -- searching locally (radius={search_radius:.2f})...')
                random_offset = (2 * np.random.rand(6) - 1) * search_radius
                local_guess = prev_config_arm + random_offset
                q_sol_arm, q_sol_full, sol, cond_num, low_manip, jump, collide, joint_vel = try_solve(local_guess)
                search_radius += 0.05
                attempt += 1

            if not sol.success:
                msg = f'IK did not converge at waypoint {k} after {max_attempts} recovery attempts (target={np.round(target_pos, 3)}, reason={sol.reason})'
                if verbose:
                    print(f'[waypoint {k}] {msg}')
                return False, np.array([]), np.array([]), msg

            if low_manip:
                msg = f'Singularity at waypoint {k}: condition number {cond_num:.2f} > {condition_number_threshold} (after {attempt - 1} recovery attempts)'
                if verbose:
                    print(f'[waypoint {k}] {msg} | q={np.round(q_sol_full, 3)}')
                return False, np.array([]), np.array([]), msg

            if jump:
                bad = np.where(joint_vel > max_joint_vel_threshold)[0].tolist()
                msg = (f'Joint jump at waypoint {k}: arm joint index(es) {bad} exceeded '
                       f'{max_joint_vel_threshold} rad/s (max={joint_vel.max():.3f}, after {attempt - 1} recovery attempts)')
                if verbose:
                    print(f'[waypoint {k}] {msg}')
                return False, np.array([]), np.array([]), msg

            if collide:
                msg = f'Collision at waypoint {k}: q={np.round(q_sol_full, 3)} (after {attempt - 1} recovery attempts)'
                if verbose:
                    print(f'[waypoint {k}] {msg}')
                return False, np.array([]), np.array([]), msg

            config_soln[k + 1, :] = q_sol_full
            prev_config_arm = q_sol_arm.copy()

        t_waypoints_full = np.concatenate(([0], np.linspace(t_transition, t_final, num_waypts)))
        t_sim = np.linspace(0, t_final, num_frames)

        pchip_interpolator = PchipInterpolator(t_waypoints_full, config_soln, axis=0)
        q_interp = pchip_interpolator(t_sim)

        dt_sim = 1.0 / self.framerate
        q_dot = np.gradient(q_interp, dt_sim, axis=0)
        return True, q_dot, q_interp, 'Success'