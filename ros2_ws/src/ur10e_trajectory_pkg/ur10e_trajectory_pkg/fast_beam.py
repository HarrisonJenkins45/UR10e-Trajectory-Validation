"""Deterministic beam policy for layered joint-state search.

The oracle supplies starts, candidates, edges and joint limits. Robot-specific
IK, collision checks, command validation, and artifact output live elsewhere.
"""
import math
import time

import numpy as np

# Declared levels, not tuned to a recording. Level 0 is what a first attempt
# costs; each later level is roughly a few times wider.
#   beam         states kept per layer
#   bank_seeds   how many of WIDE_SEED_BANK_DEG seed every layer
#   rails        rail seeds for those bank seeds: 'tracking' is the tracker's
#                rail at that layer; numbers are metres
#   start_admit  admitted lifted starts that fill the start frontier
#   start_eval   lifted starts evaluated at most, per route kind (None: all)
LEVELS = (
    {'beam': 16, 'bank_seeds': 2, 'rails': ('tracking',), 'start_admit': 16,
     'start_eval': 64},
    {'beam': 48, 'bank_seeds': 8, 'rails': ('tracking',), 'start_admit': 48,
     'start_eval': 256},
    {'beam': 128, 'bank_seeds': 8, 'rails': ('tracking', 0.5, 1.5, 2.5), 'start_admit': 128,
     'start_eval': 1024},
    {'beam': 384, 'bank_seeds': 8, 'rails': ('tracking', 0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0),
     'start_admit': None, 'start_eval': None},
)
BASE_WINDOW_LAYERS = 16
WINDOW_GROWTH = 4
BAN_HALF_WINDOW_LAYERS = 10
STATE_DECIMALS = 6


class DeadlineExpired(Exception):
    """The time budget ran out; nothing about the slice follows from that."""


def state_key(state):
    return tuple(np.round(np.asarray(state, dtype=float), STATE_DECIMALS))


def limit_room(state, lower, upper):
    """The smallest fraction of any joint's range left before a limit."""
    state = np.asarray(state, dtype=float)
    span = np.asarray(upper, dtype=float) - np.asarray(lower, dtype=float)
    room = np.minimum(state - lower, upper - state) / span
    return float(np.min(room))


def select_beam(entries, width, lower, upper):
    """Keep width entries: the cheapest half, then those with the most limit room.

    entries are (cost, key, state, parent). Deterministic: ties break on the
    state key.
    """
    ordered = sorted(entries, key=lambda e: (e[0], e[1]))
    if width is None or len(ordered) <= width:
        return ordered
    keep = ordered[:int(math.ceil(width / 2.0))]
    rest = sorted(ordered[len(keep):],
                  key=lambda e: (-limit_room(e[2], lower, upper), e[0], e[1]))
    keep += rest[:width - len(keep)]
    return sorted(keep, key=lambda e: (e[0], e[1]))


# --------------------------------------------------------------------------
# The search, independent of how candidates, edges and starts are produced
# --------------------------------------------------------------------------

class BeamSearch:
    """Incremental beam search over layers, broadened where it fails.

    oracle provides:
      start_frontier(level)                  -> ordered lifted start states
      candidates(layer, frontier_states, level) -> candidate rows
      successors(predecessor, predecessor_key, row, row_key, layer)
                                             -> [(state, step_cost)]
      limits                                 -> (lower, upper)
      rail_budget                            -> largest rail step per layer,
                                                or None to skip the prefilter
    """

    def __init__(self, oracle, num_layers, levels=LEVELS, deadline=None,
                 clock=time.perf_counter):
        self.oracle = oracle
        self.num_layers = int(num_layers)
        self.level_specs = levels
        self.max_level = len(levels) - 1
        self.deadline = deadline
        self.clock = clock
        self.levels = [0] * self.num_layers
        self.history = [None] * self.num_layers
        self.fail_counts = {}
        self.banned = {}
        self.escalations = []
        self.stats = {'layers_expanded': 0, 'searches': 0, 'max_beam': 0}
        self.restart = 0

    def check_deadline(self):
        if self.deadline is not None and self.clock() >= self.deadline:
            raise DeadlineExpired()

    def width(self, layer):
        return self.level_specs[self.levels[layer]]['beam']

    def _start(self):
        states = self.oracle.start_frontier(self.levels[0])
        entries = [(0.0, state_key(s), np.asarray(s, dtype=float), None) for s in states
                   if state_key(s) not in self.banned.get(0, set())]
        # Admitted starts arrive nearest the home first; the beam keeps that
        # order among equal (zero) costs by key, so trim in arrival order.
        width = self.width(0)
        return entries if width is None else entries[:width]

    def _expand(self, layer):
        previous = self.history[layer - 1]
        rows = self.oracle.candidates(layer, [e[2] for e in previous], self.levels[layer])
        banned = self.banned.get(layer, set())
        best = {}
        if not rows:
            return []
        row_keys = [state_key(row) for row in rows]
        rails = np.asarray([row[0] for row in rows], dtype=float)
        layer_budget = getattr(self.oracle, 'rail_budget_for_layer', None)
        budget = (layer_budget(layer) if layer_budget is not None
                  else getattr(self.oracle, 'rail_budget', None))
        pairs = []
        for index, (cost, predecessor_key, state, _) in enumerate(previous):
            # The rail step is the edge's cheapest rejection; applied to every
            # row at once it spares the per-pair call for most pairs.
            reachable = (range(len(rows)) if budget is None else
                         np.flatnonzero(np.abs(rails - state[0]) <= budget + 1e-12))
            pairs.extend((index, number) for number in reachable)
        requests = [(previous[i][2], previous[i][1], rows[n], row_keys[n]) for i, n in pairs]
        if hasattr(self.oracle, 'successors_batch'):
            answers = self.oracle.successors_batch(requests, layer)
        else:
            answers = [self.oracle.successors(*request, layer) for request in requests]
        # Folded in pair order, so the result does not depend on how the
        # answers were computed.
        for (index, _), answer in zip(pairs, answers):
            cost = previous[index][0]
            for successor, step in answer:
                key = state_key(successor)
                if key in banned:
                    continue
                total = cost + step
                held = best.get(key)
                if held is None or (total, index) < (held[0], held[3]):
                    best[key] = (total, key, np.asarray(successor, dtype=float), index)
        lower, upper = self.oracle.limits
        return select_beam(list(best.values()), self.width(layer), lower, upper)

    def _raise(self, first, last):
        changed = False
        for layer in range(first, last + 1):
            if self.levels[layer] < self.max_level:
                self.levels[layer] += 1
                changed = True
        return changed

    def _window(self, failed):
        """Raise levels before a disconnect; the layer to resume after, or None."""
        while True:
            count = self.fail_counts.get(failed, 0) + 1
            self.fail_counts[failed] = count
            span = BASE_WINDOW_LAYERS * WINDOW_GROWTH ** (count - 1)
            first = max(0, failed - span)
            changed = self._raise(first, min(failed, self.num_layers - 1))
            if changed:
                self.escalations.append({'failed_layer': failed, 'window_start': first,
                                         'levels': sorted(set(self.levels[first:failed + 1]))})
                for layer in range(first + (1 if first else 0), self.num_layers):
                    self.history[layer] = None
                return first
            if first == 0:
                return None

    def _path(self, layer):
        entries = self.history[layer]
        best = min(entries, key=lambda e: (e[0], e[1]))
        path, cost, parent = [best[2]], best[0], best[3]
        for index in range(layer - 1, -1, -1):
            entry = self.history[index][parent]
            path.append(entry[2])
            parent = entry[3]
        return list(reversed(path)), cost

    def run(self):
        """(result) with complete True and a path, or the layer it disconnected at."""
        self.stats['searches'] += 1
        while True:
            self.check_deadline()
            if self.restart == 0:
                frontier = self._start()
                if not frontier:
                    resume = self._window(0)
                    if resume is None:
                        return self._disconnected(0)
                    self.restart = resume
                    continue
                self.history[0] = frontier
                first = 1
            else:
                first = self.restart + 1
            failed = None
            for layer in range(first, self.num_layers):
                self.check_deadline()
                entries = self._expand(layer)
                self.stats['layers_expanded'] += 1
                if not entries:
                    failed = layer
                    break
                self.history[layer] = entries
                self.stats['max_beam'] = max(self.stats['max_beam'], len(entries))
            if failed is None:
                path, cost = self._path(self.num_layers - 1)
                self.restart = 0
                return {'complete': True, 'path': path, 'cost': cost,
                        'first_disconnected_layer': None}
            resume = self._window(failed)
            if resume is None:
                return self._disconnected(failed)
            self.restart = resume

    def _disconnected(self, layer):
        partial = None if layer == 0 or self.history[layer - 1] is None else self._path(layer - 1)
        return {'complete': False, 'path': None, 'cost': None,
                'first_disconnected_layer': layer,
                'last_connected_layer': None if partial is None else layer - 1,
                'partial_path': None if partial is None else partial[0]}

    def reject_path(self, path, located_layer=None):
        """Ban a path that failed validation near where it failed, and widen.

        Returns False when neither the ban nor the levels change anything, so
        another search could only find the same path again.
        """
        # Never the start: it was validated from the home, and banning it
        # would remove what every alternative must begin from.
        if located_layer is None:
            layers = range(1, len(path))
        else:
            layers = range(max(1, located_layer - BAN_HALF_WINDOW_LAYERS),
                           min(len(path), located_layer + BAN_HALF_WINDOW_LAYERS + 1))
        added = False
        for layer in layers:
            key = state_key(path[layer])
            bucket = self.banned.setdefault(layer, set())
            if key not in bucket:
                bucket.add(key)
                added = True
        raised = self._raise(0, self.num_layers - 1)
        self.escalations.append({'rejected_path_at_layer': located_layer, 'banned_new': added,
                                 'levels_raised': raised})
        self.history = [None] * self.num_layers
        self.restart = 0
        return added or raised
