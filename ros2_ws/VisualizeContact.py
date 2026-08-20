#!/usr/bin/env python3
"""Render a specific joint configuration WITH the rail/floor/wall geometry
visible and a marker at the exact contact point pybullet reported.
 
This intentionally does NOT use roboticstoolbox's PyPlot backend for shape
rendering (env.add(shape) / env.step()) -- on this Python/matplotlib/RTB
combination, RTB's internal EllipsePlot.draw() (the class it uses to draw
every added shape, not just ellipsoids) is broken in ways that don't match
what monkeypatching it can reliably intercept, since it's not clear the
patched method is even what's executing for every shape.
 
Instead this draws everything itself with plain matplotlib 3D primitives,
using exactly the same pieces we already use elsewhere in this project:
fkine_all() for link poses, trimesh for the collision meshes, and each
shape's local `.base` offset composed on top -- see check_all_collisions()
and derive_link_capsules_from_robot() in validation_core.py for the same
pattern. This has no dependency on RTB's shape-drawing internals at all,
so it isn't exposed to that version mismatch.
 
Fill in Q_FULL the same way as inspect_contact.py -- grab it from a
print(repr(q_full)) added just before check_all_collisions() in
find_feasible_segments/process_matlab_validation. Its length must match
whatever validator.robot.n currently is (still an open question from
earlier in this project -- 6 vs 7 DOF -- so double check that count
matches len(Q_FULL) or fkine_all will raise/misbehave).
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from spatialmath import SE3
import trimesh
 
from validation_core import TrajectoryValidator
 
URDF_PATH = 'ur10e.urdf'
MESH_BASE_PATH = '/Users/jenkinsh21/Desktop/GT/Research/DCSL/ASTROS_Sims/ur_description_ur10e'
 
# From your inspect_contact.py output for this pose, e.g.:
Q_FULL = np.array([0.0, -0.18586712, -0.33512837, 1.38833738, 2.13365485, -1.4793017, -1.31950443])
CONTACT_POINT = (0.7492, -0.034, 0.0225)  # or None to skip the marker
SAVE_PATH = 'contact_scene.png'  # set to None to open an interactive window instead
 
# CAMERA_EYE: camera looks toward the origin FROM this point.
CAMERA_EYE = (0.0, -0.2, 0.3)
 
# COMPOSE_LOCAL_BASE: whether to compose each collision shape's local URDF
# <collision><origin> offset (shape.base) on top of the link's own kinematic
# transform. This mirrors the same open question flagged in
# check_all_collisions() -- toggle this to match whichever setting you
# settled on there, so this render matches what the collision checker
# actually evaluated.
COMPOSE_LOCAL_BASE = True
 
 
def _get_local_base(geom):
    """Defensive lookup for a collision shape's local offset transform.
    .base isn't guaranteed to be the right attribute name across
    spatialgeometry versions -- mirrors the fallback chain your own
    _shim_shape_base() already used for exactly this reason."""
    for candidate in ('base', 'pose', 'T', 'wT', '_T', '_base'):
        val = getattr(geom, candidate, None)
        if val is not None:
            return val if hasattr(val, 'A') else SE3(val)
    return None
 
 
 
def draw_robot_meshes(ax, robot, q_full, color='steelblue', alpha=0.6, verbose=True):
    """Draw every link's collision mesh (or box, for primitive-geometry
    links like a rail) at the given configuration, using the exact same
    transform composition as check_all_collisions()."""
    transforms = robot.fkine_all(q_full)
    for link, T_link in zip(robot.links, transforms):
        shapes = getattr(link, 'collision', []) or []
        if not shapes and verbose:
            print(f'  [skip] {link.name}: no collision geometry (expected for pass-through/frame links)')
            continue
 
        for geom in shapes:
            local_base = _get_local_base(geom)
            world_T = SE3(T_link) if local_base is None else SE3(T_link) * local_base
            filename = getattr(geom, 'filename', None)
 
            if verbose:
                base_src = 'none found' if local_base is None else 'ok'
                print(f'  [draw] {link.name}: origin={world_T.t.round(3)}, local_offset={base_src}')
 
            if filename is not None:
                mesh = trimesh.load_mesh(filename)
                verts = np.asarray(mesh.vertices)
                verts_h = np.c_[verts, np.ones(len(verts))]
                verts_world = (world_T.A @ verts_h.T).T[:, :3]
                faces = np.asarray(mesh.faces)
                poly = Poly3DCollection(verts_world[faces], alpha=alpha,
                                         facecolor=color, edgecolor='none')
                ax.add_collection3d(poly)
 
            elif hasattr(geom, 'scale'):
                draw_cuboid(ax, geom.scale, world_T, color=color, alpha=alpha)
 
 
def draw_cuboid(ax, scale, world_T, color='gray', alpha=0.3):
    """Draw a box given its (sx, sy, sz) dimensions and world SE3 pose,
    as 6 filled faces (so it reads clearly from any camera angle)."""
    sx, sy, sz = np.asarray(scale, dtype=float) / 2.0
    corners_local = np.array([
        [-sx, -sy, -sz], [sx, -sy, -sz], [sx, sy, -sz], [-sx, sy, -sz],
        [-sx, -sy, sz], [sx, -sy, sz], [sx, sy, sz], [-sx, sy, sz],
    ])
    corners_h = np.c_[corners_local, np.ones(8)]
    corners_world = (world_T.A @ corners_h.T).T[:, :3]
 
    faces_idx = [
        [0, 1, 2, 3], [4, 5, 6, 7],  # bottom, top
        [0, 1, 5, 4], [2, 3, 7, 6],  # front, back
        [1, 2, 6, 5], [0, 3, 7, 4],  # right, left
    ]
    faces = [corners_world[idx] for idx in faces_idx]
    poly = Poly3DCollection(faces, alpha=alpha, facecolor=color, edgecolor='k', linewidths=0.5)
    ax.add_collection3d(poly)
    return corners_world
 
 
def draw_env_shapes(ax, env_shapes, color='gray', alpha=0.25):
    """Draw validator.env's boxes (floor, wall, rail, ...) directly from
    their known .scale + pose, no RTB shape-drawing involved."""
    for shape in env_shapes:
        scale = getattr(shape, 'scale', None)
        if scale is None:
            continue  # not a box-like shape we know how to draw generically
        pose = None
        for candidate in ('base', 'pose', 'T', '_T'):
            val = getattr(shape, candidate, None)
            if val is not None:
                pose = val if hasattr(val, 'A') else SE3(val)
                break
        if pose is None:
            pose = SE3()  # identity fallback
        draw_cuboid(ax, scale, pose, color=color, alpha=alpha)
 
 
def draw_contact_marker(ax, point, radius=0.03, color='red'):
    """Solid sphere mesh, not a scatter point -- mplot3d doesn't reliably
    depth-sort Line3DCollection scatter points against Poly3DCollection
    faces, so a small dot can render hidden behind translucent mesh
    polygons even when it should be in front. An opaque mesh collection
    sorts far more predictably against the other Poly3DCollections here."""
    u = np.linspace(0, 2 * np.pi, 20)
    v = np.linspace(0, np.pi, 20)
    x = point[0] + radius * np.outer(np.cos(u), np.sin(v))
    y = point[1] + radius * np.outer(np.sin(u), np.sin(v))
    z = point[2] + radius * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_surface(x, y, z, color=color, alpha=1.0, shade=True, zorder=100)
 
 
def set_camera_from_eye(ax, eye_xyz):
    """view_init() takes spherical angles (elev/azim), not a literal eye
    position -- convert (x, y, z) into the equivalent elev/azim so the
    camera looks toward the origin from that direction.
 
    mplot3d has a known issue exactly AT the poles (elev = +/-90, i.e. the
    eye sitting directly on the Z axis): its face depth-sorting becomes
    numerically unstable there and azim is mathematically undefined
    (arctan2(0, 0)), which produces garbled/misordered geometry -- not a
    bug in the mesh data, just this specific camera direction. Nudge a
    near-vertical eye vector slightly off-axis so elev never hits exactly
    +/-90, which avoids the degeneracy while keeping the view
    indistinguishable from directly overhead for any practical purpose.
    """
    x, y, z = eye_xyz
    EPS = 1e-3
    if abs(x) < EPS and abs(y) < EPS:
        x = EPS if x >= 0 else -EPS
        print(f'NOTE: CAMERA_EYE is on-axis (mplot3d pole singularity) -- '
              f'nudging x by {x:+.1e} to avoid degenerate rendering.')
 
    r_xy = np.hypot(x, y)
    elev = np.degrees(np.arctan2(z, r_xy))
    azim = np.degrees(np.arctan2(y, x))
    ax.view_init(elev=elev, azim=azim)
 
 
def set_equal_aspect_on_origin(ax, points):
    """Same equal-aspect fix as before, but centered on the world origin
    specifically (not the data's bounding-box center), so a requested
    camera position that 'looks at the origin' actually does."""
    points = np.asarray(points)
    max_abs = np.abs(points).max()
    half_range = max(max_abs * 1.15, 0.1)  # small margin so nothing sits exactly on the edge
    ax.set_xlim(-half_range, half_range)
    ax.set_ylim(-half_range, half_range)
    ax.set_zlim(-half_range, half_range)
 
 
def main():
    validator = TrajectoryValidator(URDF_PATH, mesh_base_path=MESH_BASE_PATH, framerate=30)
 
    if len(Q_FULL) != validator.robot.n:
        print(f'WARNING: Q_FULL has {len(Q_FULL)} elements but robot.n = {validator.robot.n} '
              f'-- fkine_all() will likely error or silently misinterpret joint indices.')
 
    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection='3d')
 
    print('Rendering robot links:')
    draw_robot_meshes(ax, validator.robot, Q_FULL, color='steelblue', alpha=0.7)
 
    env_shapes = getattr(validator, 'env', None) or []
    draw_env_shapes(ax, env_shapes, color='gray', alpha=0.25)
 
    all_points = [np.zeros(3)]  # include the origin itself in the bounding box
    transforms = validator.robot.fkine_all(Q_FULL)
    for T in transforms:
        all_points.append(SE3(T).t)
    if CONTACT_POINT is not None:
        draw_contact_marker(ax, CONTACT_POINT)
        all_points.append(np.asarray(CONTACT_POINT))
    else:
        print('CONTACT_POINT is None -- no marker drawn.')
 
    set_equal_aspect_on_origin(ax, all_points)
    set_camera_from_eye(ax, CAMERA_EYE)
 
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Contact scene (robot mesh + env geometry + contact marker)')
 
    if SAVE_PATH:
        fig.savefig(SAVE_PATH, dpi=150)
        print(f'Saved to {SAVE_PATH}')
    else:
        plt.show()
 
 
if __name__ == '__main__':
    main()