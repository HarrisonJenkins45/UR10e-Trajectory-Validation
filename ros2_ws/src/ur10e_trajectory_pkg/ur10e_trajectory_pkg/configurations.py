#!/usr/bin/env python3
"""Joint ordering and the named configurations, in one place.

Three distinct ideas were previously conflated under the name "home":

  * where the physical robot actually starts,
  * a certified planning-ready configuration,
  * the numerical initial guess handed to inverse kinematics.

Only the third has a value here yet, and it is explicitly a legacy one. The
other two are deliberately absent rather than guessed at, because a wrong
value for either is worse than no value: validating from an assumed start the
robot is not in produces confident, wrong answers.

The configuration vector and its joint names were also duplicated across the
server, the client and the Gazebo bridge, with the same numbers retyped in
each. They are defined once here.

Imports stay light on purpose -- numpy only -- so the Gazebo bridge can use
this without pulling in roboticstoolbox and pybullet.
"""
import numpy as np

# Actuated joints, in the order every configuration vector uses. The rail is
# first; everything downstream indexes it positionally, so reordering this
# silently mis-drives the robot rather than raising.
JOINT_NAMES = (
    'linear_rail_joint',
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
)

NUM_JOINTS = len(JOINT_NAMES)

# Units: the rail in METRES, the six arm joints in RADIANS. Mixing them is
# silent -- deg2rad over a whole vector turns a 1.5 m rail command into
# 0.026 m -- so anything building a configuration should say which is which.
RAIL_INDEX = 0
ARM_SLICE = slice(1, None)

# The posture carried over from the MATLAB script, as [0, -135, 90, -90, 0, 0]
# degrees for the arm with the rail at its zero end.
#
# THIS IS NOT A MEASURED START POSE. It entered the codebase because the
# simulation had no hardware feedback and something had to seed the solver.
# It is not the upstream UR description's initial posture either, which is
# [0, -90, 0, -90, 0, 0] degrees. Hardware connection is still future work, so
# nothing here has ever been compared against a real robot.
#
# It is ALSO SINGULAR: wrist_2 = 0 aligns the wrist_1 and wrist_3 axes, the
# classic UR wrist degeneracy, giving it an infinite condition number under a
# correctly computed Jacobian. See test_jacobian_contract.py.
#
# Kept, under a name that says what it is, for two reasons: it reproduces the
# historical 370/500 baseline, and the Gazebo bridge commands it at startup so
# the simulated robot does begin there. Simulation scenarios should pass it
# explicitly rather than relying on a default.
LEGACY_MATLAB_START_Q = np.deg2rad(
    np.array([0.0, 0.0, -135.0, 90.0, -90.0, 0.0, 0.0])
)

# READY_Q and DEFAULT_IK_Q0 belong here too, once stage 1's sweep has chosen
# them against the corrected Jacobian plus collision and joint-limit
# clearance. They are deliberately not defined yet: picking a convenient
# non-singular posture now would just repeat how the constant above came to
# be treated as a start pose.


def named(configuration):
    """Pair a configuration vector with its joint names, for logging."""
    configuration = np.asarray(configuration, dtype=float)
    if configuration.shape != (NUM_JOINTS,):
        raise ValueError(
            f'expected {NUM_JOINTS} joint values, got {configuration.size}'
        )
    return dict(zip(JOINT_NAMES, configuration.tolist()))
