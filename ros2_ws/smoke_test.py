#!/usr/bin/env python3
"""For a specific q_full that check_all_collisions flagged, print the exact
3D contact point(s) and penetration depth between two named links -- so you
can compare against where you actually know the rail/arm to be, instead of
just trusting the boolean.

Usage: fill in Q_FULL below with the joint vector that triggered the flag
(add a print of q_sol_full / q_full right before the check_all_collisions
call in validation_core.py if you don't already have it printed -- the
verbose collision message doesn't currently include q_full itself).
"""
import numpy as np
import pybullet as pb
from validation_core import TrajectoryValidator

URDF_PATH = 'ur10e.urdf'
MESH_BASE_PATH = '/Users/jenkinsh21/Desktop/GT/Research/DCSL/ASTROS_Sims/ur_description_ur10e'

# LINK_A = 'forearm_link'
# LINK_B = 'rail_base_link'

# Fill this in with the actual q_full that triggered the flag for point 0.
# Q_FULL = np.array([ 0.0, -0.18586712, -0.33512837,  1.38833738,  2.13365485, -1.4793017 , -1.31950443]) 


LINK_A = 'upper_arm_link'
LINK_B = 'rail_base_link'
Q_FULL= np.array([ 0. ,        -0.6525576,   0.27023249 , 0.61943729 , 2.62337345 ,-1.1857735,
 -1.16267947])

def main():
    validator = TrajectoryValidator(URDF_PATH, mesh_base_path=MESH_BASE_PATH, framerate=30)
    client = validator._pb_client
    body = validator.robot_id

    for pb_idx, q_val in zip(validator._pb_joint_indices, Q_FULL):
        pb.resetJointState(body, pb_idx, float(q_val), physicsClientId=client)
    pb.performCollisionDetection(physicsClientId=client)

    name_to_link = {v: k for k, v in validator._pb_link_name_by_index.items()}
    link_a_idx = name_to_link[LINK_A]
    link_b_idx = name_to_link[LINK_B]

    # AABBs, in world coordinates, for eyeballing which axis/region overlaps.
    for name, idx in ((LINK_A, link_a_idx), (LINK_B, link_b_idx)):
        lo, hi = pb.getAABB(body, idx, physicsClientId=client)
        print(f'{name}: world AABB lo={tuple(round(v, 3) for v in lo)} '
              f'hi={tuple(round(v, 3) for v in hi)}')

    contacts = pb.getContactPoints(bodyA=body, bodyB=body, physicsClientId=client)
    hits = [c for c in contacts if {c[3], c[4]} == {link_a_idx, link_b_idx}]

    print(f'\n{len(hits)} contact point(s) between {LINK_A} and {LINK_B}:')
    for c in hits:
        pos_on_a = tuple(round(v, 4) for v in c[5])
        pos_on_b = tuple(round(v, 4) for v in c[6])
        normal = tuple(round(v, 4) for v in c[7])
        depth = round(-c[8], 4)  # contactDistance is negative when penetrating
        print(f'  world point on {LINK_A}: {pos_on_a}')
        print(f'  world point on {LINK_B}: {pos_on_b}')
        print(f'  normal: {normal}, penetration depth: {depth} m')


if __name__ == '__main__':
    main()