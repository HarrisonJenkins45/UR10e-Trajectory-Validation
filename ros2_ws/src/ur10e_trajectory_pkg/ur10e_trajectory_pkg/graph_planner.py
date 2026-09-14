#!/usr/bin/env python3
"""Layered graph over per-waypoint candidates, and its first gate.

One binary question before any cost tuning: does a complete path exist from
the actual q_start through all 500 layers?

State is the full LIFTED configuration, equivalently
(layer, canonical candidate, winding vector). The winding has to persist in
the state rather than sitting on the edge: which lifts a successor can reach
depends on where the predecessor actually is, so an edge-local lift would
lose exactly the information the next transition needs.

Edges are built cheapest-check-first, because collision queries dominate:

  1. rail displacement            one subtraction
  2. reachable lifts and velocity closed form per joint
  3. swept collision              a handful of PyBullet queries
  4. cost

Two builds, because a single one cannot separate a graph defect from an
incomplete candidate set:

  validation  the corrected greedy configuration is injected at every layer
              with provenance tracking_oracle. A complete path MUST exist,
              every reference edge must survive lifting, and the optimal cost
              must not exceed the injected path's
  generator   injected nodes excluded, testing whether independent candidate
              generation suffices, and naming the first disconnected layer
              when it does not

Acceleration, jerk and continuous singularity margin are deliberately absent.
They belong to the trajectory optimisation that follows, not in first-order
graph state.
"""
import argparse
import json
import sys

import numpy as np

from ur10e_trajectory_pkg import environment
from ur10e_trajectory_pkg.configurations import (
    ARM_SLICE,
    LEGACY_MATLAB_START_Q,
    NUM_JOINTS,
    PERIODIC_JOINTS,
    RAIL_INDEX,
)
from ur10e_trajectory_pkg.joint_coordinates import reachable_feasible_lifts

SCHEMA_VERSION = 1

# Duration of the approach from q_start to the first waypoint. This is NOT the
# 0.1 s trajectory step: the arm has to get to the trajectory's start, and
# charging that move at the per-waypoint rate would reject it as a velocity
# violation for no physical reason.
TRANSITION_SECONDS = 2.0

# Interior samples per edge for swept collision. The endpoints are already
# known collision-free as nodes, so this asks only about the space between.
SWEPT_SAMPLES = 3

# States are keyed on rounded coordinates so that two arithmetically distinct
# routes to the same configuration share a state instead of splitting the
# frontier.
STATE_DECIMALS = 6


def state_key(configuration):
    return tuple(np.round(np.asarray(configuration, dtype=float), STATE_DECIMALS))


def edge_cost(predecessor, successor, velocity_limits, dt):
    """Normalised squared motion, each joint against its own budget.

    Dividing by velocity_limit * dt puts the rail's metres and the arm's
    radians on one scale, so the sum is meaningful rather than an accidental
    unit mixture.
    """
    delta = np.asarray(successor) - np.asarray(predecessor)
    budget = np.asarray(velocity_limits) * dt
    return float(np.sum((delta / budget) ** 2))


class LayeredGraph:
    def __init__(self, validator, candidates, num_layers,
                 dt=0.1, transition_seconds=TRANSITION_SECONDS,
                 arm_velocity_limit=2.0, swept_samples=SWEPT_SAMPLES):
        self.validator = validator
        self.candidates = candidates
        self.num_layers = num_layers
        self.dt = dt
        self.transition_seconds = transition_seconds
        self.swept_samples = swept_samples

        self.limits = (validator.robot.qlim[0], validator.robot.qlim[1])
        self.velocity_limits = np.array(
            [validator._rail_vel_limit] + [arm_velocity_limit] * 6)
        self.counters = {
            'pairs_considered': 0,
            'pruned_rail': 0,
            'pruned_velocity_or_lift': 0,
            'pruned_swept_collision': 0,
            'edges_built': 0,
        }

    def swept_collision(self, predecessor, successor):
        """Sample between two configurations, both already valid as nodes.

        Linear in joint space, which is what the PCHIP interpolation
        approximates between adjacent waypoints. Last in the pruning order
        because it is the only step that costs a physics query.
        """
        for fraction in np.linspace(0.0, 1.0, self.swept_samples + 2)[1:-1]:
            between = predecessor + fraction * (successor - predecessor)
            if self.validator.check_all_collisions(between):
                return True
        return False

    def successors(self, predecessor, layer, dt):
        """Every reachable lifted successor at this layer, with its cost."""
        budget = self.velocity_limits * dt
        out = []
        for candidate in self.candidates[layer]:
            self.counters['pairs_considered'] += 1

            # 1. rail displacement, the cheapest possible rejection
            if abs(candidate[RAIL_INDEX] - predecessor[RAIL_INDEX]) > budget[RAIL_INDEX]:
                self.counters['pruned_rail'] += 1
                continue

            # 2. reachable lifts, closed form per joint
            lifts = reachable_feasible_lifts(
                candidate[ARM_SLICE], predecessor[ARM_SLICE],
                (self.limits[0][ARM_SLICE], self.limits[1][ARM_SLICE]),
                PERIODIC_JOINTS[ARM_SLICE],
                self.velocity_limits[ARM_SLICE], dt)
            if not lifts:
                self.counters['pruned_velocity_or_lift'] += 1
                continue

            for arm in lifts:
                successor = np.concatenate(([candidate[RAIL_INDEX]], arm))

                # 3. swept collision, the only physics query
                if self.swept_collision(predecessor, successor):
                    self.counters['pruned_swept_collision'] += 1
                    continue

                # 4. cost
                self.counters['edges_built'] += 1
                out.append((successor,
                            edge_cost(predecessor, successor,
                                      self.velocity_limits, dt)))
        return out

    def shortest_path(self, q_start):
        """Dynamic programming over the layered DAG.

        Edges only ever go from layer i to i+1, so one sweep per layer keeping
        the best cost per state is exact. Only the current frontier is held,
        which bounds memory regardless of depth.
        """
        frontier = {state_key(q_start): (0.0, np.asarray(q_start, float), None)}
        history = [frontier]
        first_empty = None

        for layer in range(self.num_layers):
            dt = self.transition_seconds if layer == 0 else self.dt
            nxt = {}
            for _, (cost, configuration, _) in frontier.items():
                for successor, step in self.successors(configuration, layer, dt):
                    key = state_key(successor)
                    total = cost + step
                    if key not in nxt or total < nxt[key][0]:
                        nxt[key] = (total, successor, state_key(configuration))
            if not nxt:
                first_empty = layer
                break
            frontier = nxt
            history.append(frontier)

        if first_empty is not None:
            return None, first_empty, history

        final = min(frontier.items(), key=lambda item: item[1][0])
        path = [final[1][1]]
        key = final[1][2]
        for level in range(len(history) - 2, -1, -1):
            cost, configuration, parent = history[level][key]
            path.append(configuration)
            key = parent
        return (list(reversed(path)), final[1][0]), None, history


def load_candidates(path, num_layers, include_oracle):
    """Canonical configurations per layer, optionally with injected nodes."""
    with open(path, encoding='utf-8') as handle:
        document = json.load(handle)
    layers = []
    for index in range(num_layers):
        entries = document['candidates'].get(str(index), [])
        rows = []
        for entry in entries:
            if not include_oracle and entry.get('provenance_tag') == 'tracking_oracle':
                continue
            rows.append(np.array([entry['rail_position'],
                                  *entry['q_arm_canonical']], dtype=float))
        layers.append(rows)
    return layers, document


def inject_oracle(layers, oracle_path):
    """Add the corrected greedy configuration to every layer.

    Without this a failure to reproduce the greedy path is ambiguous: it could
    be a graph defect or an incomplete candidate set. With it, the graph is
    known to contain a valid path, so any failure is the graph's.
    """
    for index, configuration in enumerate(oracle_path):
        layers[index] = list(layers[index]) + [np.asarray(configuration, float)]
    return layers


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidates', required=True)
    parser.add_argument('--urdf', default='/root/ros2_ws/ur10e.urdf')
    parser.add_argument('--layers', type=int, default=500)
    parser.add_argument('--build', choices=['validation', 'generator'],
                        default='validation')
    parser.add_argument('--out', default='graph.json')
    args = parser.parse_args(argv)

    from ament_index_python.packages import get_package_share_directory

    from ur10e_trajectory_pkg.candidate_generator import tracking_reference_path
    from ur10e_trajectory_pkg.failure_census import (
        _validator,
        load_trajectory,
        run_tracking,
    )

    mesh_path = get_package_share_directory('ur_description')
    validator = _validator(args.urdf, mesh_path)
    targets, quaternions, dt, _ = load_trajectory(None, args.layers,
                                                  with_metadata=True)
    layers, _ = load_candidates(args.candidates, args.layers,
                                include_oracle=False)

    if args.build == 'validation':
        records, _ = run_tracking(validator, targets, quaternions, dt,
                                  LEGACY_MATLAB_START_Q)
        committed = {}
        for record in records:
            if record['accepted']:
                committed.setdefault(record['waypoint_index'], record['q_full'])
        oracle = [committed.get(index) for index in range(args.layers)]
        if any(step is None for step in oracle):
            print('greedy tracking does not cover every layer; '
                  'the validation build needs a complete reference')
            return 1
        layers = inject_oracle(layers, oracle)

    graph = LayeredGraph(validator, layers, args.layers, dt=dt)
    result, first_empty, _ = graph.shortest_path(LEGACY_MATLAB_START_Q)

    document = {
        'schema_version': SCHEMA_VERSION,
        'environment': environment.describe(),
        'build': args.build,
        'layers': args.layers,
        'counters': graph.counters,
        'complete_path': result is not None,
        'first_disconnected_layer': first_empty,
        'path_cost': None if result is None else result[1],
        'path': None if result is None else [c.tolist() for c in result[0]],
    }
    with open(args.out, 'w', encoding='utf-8') as handle:
        json.dump(document, handle, indent=1, sort_keys=True)

    print(f"build={args.build}  complete_path={document['complete_path']}")
    if result is None:
        print(f'first disconnected layer: {first_empty}')
    else:
        print(f'path cost: {result[1]:.4f} over {len(result[0])} states')
    for name, value in graph.counters.items():
        print(f'  {name:26s} {value}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
