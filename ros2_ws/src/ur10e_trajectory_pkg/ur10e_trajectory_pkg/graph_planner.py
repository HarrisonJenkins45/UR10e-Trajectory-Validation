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

Full run, 500 layers, both builds, under the validator's own limits (URDF
per-joint arm values, rail capped at 1.0 m/s):

    build        complete  cost     pairs      rail-pruned  edges
    validation   yes       1.0055   4,830,414  3,683,412    957,698
    generator    yes       1.0055   4,591,681  3,472,076    938,059

    greedy tracker, same cost function            1.1082

Two results worth keeping. The generator-only build reaches the SAME optimum
as the build with the greedy path injected, so independent candidate
generation is sufficient on this trajectory and the oracle added nothing it
did not already contain. And the graph path costs 9.3% less than the greedy
tracker's, which is global branch choice buying something a greedy tracker
cannot, on a trajectory where both succeed.

Swept collision rejected 74 edges of 957,698. Cheap kinematic pruning removed
76% of pairs before any physics query, which is what makes a dense build
affordable at this size.

Costs are only comparable under the same velocity limits, since the limits
ARE the normalisation. An earlier run used a uniform 2.0 rad/s arm cap and
recorded 2.1761 against a greedy 2.3310; the path it chose is identical in all
501 states to the one above, but those figures cannot be set beside these.

The frontier stays bounded rather than growing with depth: 4.8M pairs over
500 layers is under 10,000 per layer against roughly 68 candidates. No beam or
dominance rule is needed at this scale.
"""
import argparse
import json
import sys

import numpy as np

from ur10e_trajectory_pkg import environment, motion_limits
from ur10e_trajectory_pkg.configurations import (
    ARM_SLICE,
    LEGACY_MATLAB_START_Q,
    NUM_JOINTS,
    PERIODIC_JOINTS,
    RAIL_INDEX,
)
from ur10e_trajectory_pkg.joint_coordinates import reachable_feasible_lifts

# 2: records the effective velocity limits and the greedy reference cost.
SCHEMA_VERSION = 2

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


# Declared before any placement's executed path was measured: the task counts
# as starting at rest if every joint's entry velocity is within this fraction
# of its velocity limit AND its entry acceleration within this fraction of its
# acceleration limit. A recorded check here; a constraint only if needed.
AT_REST_TOLERANCE_FRACTION = 0.02


def entry_state_report(path, dt, velocity_limits, acceleration_limits,
                       tolerance=AT_REST_TOLERANCE_FRACTION):
    """Per-joint entry velocity and acceleration of an executed path.

    From the first three states, as PCHIP derives them. The warmup ends at
    rest, so a joint entering above tolerance meets it with a step. With a
    spin-up the task's tool motion at the first waypoints is essentially zero,
    so any entry motion is the redundancy -- the rail sliding while the tool
    holds still -- which is a planner choice rather than a task demand.
    """
    from ur10e_trajectory_pkg.configurations import JOINT_NAMES
    from ur10e_trajectory_pkg.ready_pose_sweep import entry_state_from_prefix

    velocity, acceleration = entry_state_from_prefix(np.asarray(path[:3], float), dt)
    velocity_ratio = np.abs(velocity) / np.asarray(velocity_limits, float)
    acceleration_ratio = np.abs(acceleration) / np.asarray(acceleration_limits, float)
    worst = np.maximum(velocity_ratio, acceleration_ratio)
    dominant = int(np.argmax(worst))
    return {
        'velocity': velocity.tolist(),
        'acceleration': acceleration.tolist(),
        'velocity_ratio': velocity_ratio.tolist(),
        'acceleration_ratio': acceleration_ratio.tolist(),
        'max_velocity_ratio': float(np.max(velocity_ratio)),
        'max_acceleration_ratio': float(np.max(acceleration_ratio)),
        'dominant_joint': JOINT_NAMES[dominant],
        'dominant_kind': ('velocity' if velocity_ratio[dominant] >= acceleration_ratio[dominant]
                          else 'acceleration'),
        'tolerance_fraction': tolerance,
        'at_rest': bool(np.all(velocity_ratio <= tolerance)
                        and np.all(acceleration_ratio <= tolerance)),
        'joints_over_tolerance': [JOINT_NAMES[i] for i in range(len(worst))
                                  if worst[i] > tolerance],
    }


def path_cost(q_start, path, velocity_limits, dt,
              transition_seconds=TRANSITION_SECONDS):
    """q_start None means a free start: the path's own first state, no
    transition edge charged."""
    if q_start is None:
        return (_path_cost_from(path[0], path[1:], velocity_limits, dt)
                if len(path) > 1 else 0.0)
    return _path_cost_fixed(q_start, path, velocity_limits, dt,
                            transition_seconds)


def _path_cost_from(start, path, velocity_limits, dt):
    total, previous = 0.0, np.asarray(start, dtype=float)
    for configuration in path:
        total += edge_cost(previous, configuration, velocity_limits, dt)
        previous = np.asarray(configuration, dtype=float)
    return total


def _path_cost_fixed(q_start, path, velocity_limits, dt,
                     transition_seconds=TRANSITION_SECONDS):
    """Total edge cost of a path under the graph's own cost function.

    What makes a greedy path and a graph path comparable: the same limits and
    the same normalisation. Costs computed under different velocity limits are
    NOT comparable, since the limits are the normalisation.
    """
    total, previous = 0.0, np.asarray(q_start, dtype=float)
    for index, configuration in enumerate(path):
        step = transition_seconds if index == 0 else dt
        total += edge_cost(previous, configuration, velocity_limits, step)
        previous = np.asarray(configuration, dtype=float)
    return total


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
                 velocity_limits=None, swept_samples=SWEPT_SAMPLES):
        self.validator = validator
        self.candidates = candidates
        self.num_layers = num_layers
        self.dt = dt
        self.transition_seconds = transition_seconds
        self.swept_samples = swept_samples

        self.limits = (validator.robot.qlim[0], validator.robot.qlim[1])
        # The validator's own limits unless told otherwise: the URDF's
        # per-joint arm values and the rail's capped value. A uniform arm
        # figure here once made the graph prune, lift and cost against a
        # different robot than the tracker it is compared with.
        self.velocity_limits = (
            np.asarray(validator.velocity_limits, dtype=float)
            if velocity_limits is None
            else np.asarray(velocity_limits, dtype=float))
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

    def start_states(self, lifts=False):
        """Every layer-0 candidate as a start, optionally in every legal winding.

        Canonical only by default. Each winding of a start is a distinct graph
        state all the way down, so including them all can multiply the
        frontier by the number of lifts (up to 2 per periodic arm joint).
        """
        from itertools import product

        from ur10e_trajectory_pkg.joint_coordinates import feasible_lifts

        states = []
        for candidate in self.candidates[0]:
            candidate = np.asarray(candidate, dtype=float)
            if not lifts:
                states.append(candidate)
                continue
            options = [feasible_lifts(value, self.limits[0][ARM_SLICE][i],
                                      self.limits[1][ARM_SLICE][i],
                                      PERIODIC_JOINTS[ARM_SLICE][i])
                       for i, value in enumerate(candidate[ARM_SLICE])]
            for arm in product(*options):
                states.append(np.concatenate(([candidate[RAIL_INDEX]], arm)))
        return states

    def shortest_path(self, q_start=None, start_lifts=False):
        """Dynamic programming over the layered DAG.

        Edges only ever go from layer i to i+1, so one sweep per layer keeping
        the best cost per state is exact. Only the current frontier is held,
        which bounds memory regardless of depth.

        q_start None is a FREE start: a common source connects to every
        layer-0 candidate at zero cost, so the planner chooses where the task
        begins, and the path's first state is that choice. The arm is brought
        there by a separate warmup command, so no transition edge exists and
        the path has num_layers states. With a q_start, the path begins there
        and the first edge is the transition, as before.
        """
        first_empty = None
        if q_start is None:
            frontier = {}
            for state in self.start_states(start_lifts):
                frontier.setdefault(state_key(state), (0.0, state, None))
            self.counters['start_states'] = len(frontier)
            history = [frontier]
            if not frontier:
                return None, 0, history
            layers = range(1, self.num_layers)
        else:
            frontier = {state_key(q_start): (0.0, np.asarray(q_start, float), None)}
            history = [frontier]
            layers = range(self.num_layers)

        for layer in layers:
            dt = self.transition_seconds if (layer == 0 and q_start is not None) else self.dt
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
    parser.add_argument('--layers', type=int, default=500,
                        help='RECORDED waypoints; a spin-up adds samples')
    parser.add_argument('--build', choices=['validation', 'generator'],
                        default='validation')
    parser.add_argument('--out', default='graph.json')
    parser.add_argument('--start', choices=['legacy', 'free'], default='legacy',
                        help='legacy: from LEGACY_MATLAB_START_Q through a '
                             'transition edge; free: the planner chooses the '
                             'first-waypoint candidate')
    parser.add_argument('--start-lifts', action='store_true',
                        help='free start in every legal winding, not only '
                             'canonical')
    parser.add_argument('--spin-up-s', type=float, default=None,
                        help='spin-up the candidates were generated with')
    parser.add_argument('--placement', default='nominal',
                        help='envelope placement the candidates were generated at')
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
    from ur10e_trajectory_pkg.failure_census import placement_RG_for
    placement_RG = (None if args.placement == 'nominal'
                    else placement_RG_for(args.placement))
    targets, quaternions, dt, trajectory_metadata = load_trajectory(
        None, args.layers, with_metadata=True, spin_up_s=args.spin_up_s,
        placement_RG=placement_RG)
    num_layers = len(targets)
    layers, candidates_document = load_candidates(args.candidates, num_layers,
                                                  include_oracle=False)
    generated_RG = (candidates_document.get('manifest', {})
                    .get('trajectory', {}).get('placement_RG'))
    if generated_RG is None or not np.allclose(generated_RG,
                                               trajectory_metadata['placement_RG'],
                                               atol=1e-9):
        print(f'candidates were not generated at placement {args.placement!r}')
        return 1
    if candidates_document.get('num_waypoints') != num_layers:
        print(f"candidates cover {candidates_document.get('num_waypoints')} "
              f'waypoints but this trajectory has {num_layers}; generate them '
              'with the same --waypoints and --spin-up-s')
        return 1
    q_start = LEGACY_MATLAB_START_Q if args.start == 'legacy' else None

    reference_cost = None
    if args.build == 'validation':
        records, _ = run_tracking(validator, targets, quaternions, dt,
                                  LEGACY_MATLAB_START_Q)
        committed = {}
        for record in records:
            if record['accepted']:
                committed.setdefault(record['waypoint_index'], record['q_full'])
        oracle = [committed.get(index) for index in range(num_layers)]
        if any(step is None for step in oracle):
            print('greedy tracking does not cover every layer; '
                  'the validation build needs a complete reference')
            return 1
        layers = inject_oracle(layers, oracle)

    graph = LayeredGraph(validator, layers, num_layers, dt=dt)
    if args.build == 'validation':
        reference_cost = path_cost(q_start, oracle, graph.velocity_limits, dt)
    result, first_empty, _ = graph.shortest_path(q_start,
                                                 start_lifts=args.start_lifts)

    document = {
        'schema_version': SCHEMA_VERSION,
        'environment': environment.describe(),
        'build': args.build,
        'layers': num_layers,
        'recorded_waypoints': args.layers,
        'spin_up': trajectory_metadata.get('spin_up'),
        'start_mode': args.start,
        'start_lifts': args.start_lifts,
        'q_start': None if q_start is None else np.asarray(q_start).tolist(),
        'chosen_start': (None if result is None
                         else np.asarray(result[0][0]).tolist()),
        'placement': args.placement,
        'placement_RG': np.asarray(trajectory_metadata['placement_RG']).tolist(),
        'entry_state': (None if result is None else entry_state_report(
            result[0], dt, graph.velocity_limits,
            motion_limits.acceleration_vector())),
        'counters': graph.counters,
        'velocity_limits': motion_limits.effective_limits(validator),
        'edge_velocity_limits': graph.velocity_limits.tolist(),
        'transition_seconds': (graph.transition_seconds if q_start is not None
                               else None),
        'greedy_reference_cost': reference_cost,
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
        print(f'start ({args.start}): {np.round(result[0][0], 4).tolist()}')
        entry = document['entry_state']
        print(f"entry state: at_rest={entry['at_rest']} max velocity ratio "
              f"{entry['max_velocity_ratio']:.5f} max acceleration ratio "
              f"{entry['max_acceleration_ratio']:.5f} dominant "
              f"{entry['dominant_joint']} ({entry['dominant_kind']})")
    if reference_cost is not None:
        print(f'greedy tracker cost, same function: {reference_cost:.4f}')
    for name, value in graph.counters.items():
        print(f'  {name:26s} {value}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
