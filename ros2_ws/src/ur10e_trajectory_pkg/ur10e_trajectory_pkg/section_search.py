#!/usr/bin/env python3
"""Where to probe a recording for its longest certified section, and when to stop.

The policy only: which slice [start, start + samples) to certify next, what a
certification's outcome licenses, and what the finite search did and did not
cover. It never runs the pipeline itself -- a certifier is passed in -- so the
policy is tested with controlled fakes and the command (section_planner) owns
the processes, the artifacts and the resources.

Definitions
-----------
  section    recorded samples [start, end), half-open, planned as a slice of
             its own (own targets, own spin-up, own candidates, own graph)
  duration   timestamps[end - 1] - timestamps[start]: recorded motion only,
             from the recorded timestamps, never an assumed rate. Warmup and
             spin-up are excluded
  probe      one certification of one slice. It passes only when every stage
             of the pipeline passes on that slice

What an outcome licenses
------------------------
  pass                 that slice is certified. Nothing about longer slices
  graph disconnect     an upper bound for THAT start: candidates are generated
                       per waypoint from causal seeds and the graph keeps its
                       whole frontier, so a slice with more samples than the
                       graph covered disconnects at the same layer. Only a
                       disconnect under the default start policy counts (not
                       the strict-winding backstop), with via poses tried when
                       they were offered, on every spin-up rung tried
  any other failure    evidence about that slice only. A longer slice gets a
                       different shortest path and different refinement, so a
                       refinement or continuous failure bounds nothing; it can
                       only suggest a shorter length to try
  resource limit       nothing at all, and the report says the probe was cut

No failure at one start says anything about another start: each start has its
own targets, spin-up, candidates and lifted starts. Starts are therefore never
pruned by another start's result or by a heuristic; heuristics (hints) only
reorder starts within one refinement level of the coverage grid.

Search
------
  starts     a deterministic coverage order over every start that leaves
             room for the target: the recording's first and last feasible
             starts, then midpoints, level by level, down to a declared
             stride. Hinted starts are added (never substituted) right after
             the first level
  target     until a section of at least the target is certified, each start
             in order is probed at the target length (or --first-probe-s).
             A failed probe may shrink toward the target on its own hints,
             never below it
  extend     once one is certified, the remaining budget seeks a longer one:
             the longest certified start is grown geometrically until a
             failure, then bisected down to the length resolution; then the
             other certified starts; then unprobed starts, at the shortest
             length that would beat the best; then starts whose failures
             bound nothing, at that length
  budget     probes, probed samples and probe hours, each declared. Consumed
             budget is counted from the journal, so a resumed search makes the
             same decisions and continues where it stopped
"""
import math
from dataclasses import dataclass, field

import numpy as np

TARGET_SECTION_S = 60.0

# Declared defaults, not tuned to any recording.
DEFAULT_START_STRIDE_S = 10.0
DEFAULT_LENGTH_RESOLUTION_S = 1.0
DEFAULT_GROWTH = 2.0
DEFAULT_MAX_PROBES_PER_START = 12
# A failure located in time ends the next attempt this many layers before it:
# refinement smooths across neighbouring waypoints, so ending exactly at the
# failing one would leave the smoothing the same room to push it over.
LOCATED_FAILURE_MARGIN_LAYERS = 5
TIME_EPS_S = 1e-9

PHASE_TARGET = 'target'
PHASE_EXTEND = 'extend'


# --------------------------------------------------------------------------
# Time, from the recorded timestamps
# --------------------------------------------------------------------------

class Timeline:
    """Recorded timestamps, and every conversion between samples and seconds."""

    def __init__(self, timestamps):
        times = np.asarray(timestamps, dtype=float)
        if times.ndim != 1 or len(times) < 2:
            raise ValueError('a timeline needs at least two timestamps')
        if not np.all(np.isfinite(times)) or np.any(np.diff(times) <= 0.0):
            raise ValueError('timestamps must be finite and strictly increasing')
        self.times = times

    def __len__(self):
        return len(self.times)

    @property
    def step_s(self):
        """The median step, used only to express a stride in samples."""
        return float(np.median(np.diff(self.times)))

    def duration(self, start, samples):
        """Recorded motion over [start, start + samples)."""
        return float(self.times[int(start) + int(samples) - 1] - self.times[int(start)])

    def samples_for(self, start, seconds):
        """Fewest samples from start spanning at least seconds, or None."""
        start = int(start)
        wanted = self.times[start] + float(seconds) - TIME_EPS_S
        end = int(np.searchsorted(self.times, wanted, side='left'))
        if end >= len(self.times):
            return None
        return max(end - start + 1, 2)

    def last_start(self, seconds):
        """The last start from which seconds of recorded motion remain, or None."""
        wanted = self.times[-1] - float(seconds) + TIME_EPS_S
        index = int(np.searchsorted(self.times, wanted, side='right')) - 1
        return index if index >= 0 else None

    def interval(self, start, samples):
        start, samples = int(start), int(samples)
        return {'start_index': start, 'end_index': start + samples, 'samples': samples,
                'half_open': True,
                'start_time_s': float(self.times[start]),
                'last_sample_time_s': float(self.times[start + samples - 1]),
                # Rounded to the tolerance durations are compared at, so a
                # 60 s section does not print as 59.99999999999994.
                'recorded_duration_s': round(self.duration(start, samples), 9),
                'duration_tolerance_s': TIME_EPS_S}


def samples_through_layer(layer, samples_added):
    """Recorded samples a slice's layers 0..layer cover.

    The spin-up warps the first 2h layers over the first h recorded samples;
    from layer 2h on, layer j is recorded sample j - h.
    """
    h = int(samples_added)
    if layer < 2 * h:
        return 1
    return int(layer) - h + 1


# --------------------------------------------------------------------------
# Starts
# --------------------------------------------------------------------------

def coverage_levels(last_start, stride):
    """Starts 0..last_start, coarse to fine, never closer than stride.

    Level 0 is the first and last start; every later level holds the midpoints
    of the gaps the previous levels left, so stopping after any level leaves
    starts spread over the whole recording, not bunched at its beginning.
    """
    last_start, stride = int(last_start), max(int(stride), 1)
    levels = [[0] if last_start == 0 else [0, last_start]]
    gaps = [] if last_start == 0 else [(0, last_start)]
    while gaps:
        level, next_gaps = [], []
        for low, high in gaps:
            middle = (low + high) // 2
            if middle - low >= stride and high - middle >= stride:
                level.append(middle)
                next_gaps += [(low, middle), (middle, high)]
        if level:
            levels.append(level)
        gaps = next_gaps
    return levels


def start_order(last_start, stride, hints=None):
    """Every start the search will consider, in order, with its level.

    hints maps start -> score (higher first). A hint reorders starts within a
    level and may add starts off the grid (right after level 0); it never
    removes one.
    """
    hints = {int(k): float(v) for k, v in (hints or {}).items()}
    levels = coverage_levels(last_start, stride)
    on_grid = {s for level in levels for s in level}
    extra = sorted((s for s in hints if 0 <= s <= last_start and s not in on_grid),
                   key=lambda s: (-hints[s], s))

    def ranked(level):
        return sorted(level, key=lambda s: (-hints.get(s, -math.inf), s))

    order = [(s, 0, False) for s in ranked(levels[0])]
    order += [(s, 'hinted', True) for s in extra]
    for number, level in enumerate(levels[1:], start=1):
        order += [(s, number, s in hints) for s in ranked(level)]
    return order, levels


# --------------------------------------------------------------------------
# Outcomes, budget, state
# --------------------------------------------------------------------------

def outcome(start, samples, passed, classification=None, graph_bound_samples=None,
            shrink_hint_samples=None, resource_limited=False, seconds=0.0, **extra):
    """A probe outcome as the policy reads it. Extra keys are carried through."""
    record = {'start': int(start), 'samples': int(samples), 'passed': bool(passed),
              'classification': classification or {'class': 'pass' if passed
                                                   else 'unclassified', 'detail': None},
              'graph_bound_samples': (None if graph_bound_samples is None
                                      else int(graph_bound_samples)),
              'shrink_hint_samples': (None if shrink_hint_samples is None
                                      else int(shrink_hint_samples)),
              'resource_limited': bool(resource_limited), 'seconds': float(seconds)}
    record.update(extra)
    return record


@dataclass
class Budget:
    """The declared compute budget. None means unlimited on that axis."""
    probes: int = 10
    probe_samples: int = None
    probe_hours: float = None

    def as_dict(self):
        return {'probes': self.probes, 'probe_samples': self.probe_samples,
                'probe_hours': self.probe_hours}


@dataclass
class SearchConfig:
    target_s: float = TARGET_SECTION_S
    first_probe_s: float = None
    max_probe_samples: int = 2000
    start_stride_s: float = DEFAULT_START_STRIDE_S
    length_resolution_s: float = DEFAULT_LENGTH_RESOLUTION_S
    growth: float = DEFAULT_GROWTH
    max_probes_per_start: int = DEFAULT_MAX_PROBES_PER_START
    budget: Budget = field(default_factory=Budget)
    hints: dict = None

    def as_dict(self):
        return {'target_s': self.target_s, 'first_probe_s': self.first_probe_s,
                'max_probe_samples': self.max_probe_samples,
                'start_stride_s': self.start_stride_s,
                'length_resolution_s': self.length_resolution_s, 'growth': self.growth,
                'max_probes_per_start': self.max_probes_per_start,
                'budget': self.budget.as_dict(),
                'hints': None if not self.hints else {str(k): v for k, v
                                                      in sorted(self.hints.items())}}


class StartState:
    """Everything the probes at one start have established."""

    def __init__(self, start, available):
        self.start = int(start)
        self.available = int(available)     # samples to the end of the recording
        self.probes = []

    @property
    def certified(self):
        passed = [p['samples'] for p in self.probes if p['passed']]
        return max(passed) if passed else 0

    @property
    def graph_bound(self):
        bounds = [p['graph_bound_samples'] for p in self.probes
                  if not p['passed'] and p['graph_bound_samples'] is not None]
        return min(bounds) if bounds else None

    def ceiling(self, cap):
        """The longest slice this start may still be probed at."""
        ceiling = min(self.available, int(cap))
        if self.graph_bound is not None:
            ceiling = min(ceiling, self.graph_bound)
        return ceiling

    def probed(self, samples):
        return any(p['samples'] == int(samples) for p in self.probes)

    def failures_above(self, samples):
        return sorted((p for p in self.probes
                       if not p['passed'] and p['samples'] > samples),
                      key=lambda p: p['samples'])


class SectionSearch:
    """The deterministic search. run(certify) drives it to a stop."""

    def __init__(self, timeline, config):
        self.timeline = timeline
        self.config = config
        self.target_s = float(config.target_s)
        last = timeline.last_start(self.target_s)
        self.last_start = last
        stride = max(1, int(round(float(config.start_stride_s) / timeline.step_s)))
        self.stride_samples = stride
        if last is None:
            self.order, self.levels = [], []
        else:
            self.order, self.levels = start_order(last, stride, config.hints)
        self.starts = {s: StartState(s, len(timeline) - s) for s, _, _ in self.order}
        self.probes = []
        self.stop_reason = None
        self.next_planned = None
        self.decisions = []

    # ---- lengths ---------------------------------------------------------

    def target_samples(self, start):
        return self.timeline.samples_for(start, self.target_s)

    def resolution_samples_between(self, start, shorter, longer):
        """True when the two lengths differ by no more than the resolution."""
        return (self.timeline.duration(start, longer)
                - self.timeline.duration(start, shorter)
                <= float(self.config.length_resolution_s) + TIME_EPS_S)

    @property
    def best(self):
        """(start, samples) of the longest certified section, earliest on ties."""
        best = None
        for state in sorted(self.starts.values(), key=lambda s: s.start):
            if state.certified and (best is None or self.timeline.duration(
                    state.start, state.certified) > self.timeline.duration(*best)
                    + TIME_EPS_S):
                best = (state.start, state.certified)
        return best

    def target_met(self):
        best = self.best
        return best is not None and self.timeline.duration(*best) >= self.target_s - TIME_EPS_S

    # ---- proposals -------------------------------------------------------

    def _shrink(self, state, failed, floor):
        """A shorter length a failed probe suggests, strictly above floor."""
        hint = failed['shrink_hint_samples']
        if failed['graph_bound_samples'] is not None:
            hint = failed['graph_bound_samples'] if hint is None else min(
                hint, failed['graph_bound_samples'])
        if hint is not None and floor < hint < failed['samples'] and not state.probed(hint):
            return hint, 'shrink to the failed probe\'s own evidence'
        if hint is not None and hint <= floor:
            # The failure's own evidence ends at or before what is already
            # certified, so halfway to it is the least likely length to pass:
            # step a quarter of the gap past the certified length instead.
            step = floor + max(1, int(math.ceil((failed['samples'] - floor) / 4.0)))
            if floor < step < failed['samples'] and not state.probed(step):
                return step, 'step past evidence that stops at the certified length'
        middle = (floor + failed['samples']) // 2
        if floor < middle < failed['samples'] and not state.probed(middle):
            return middle, 'bisect below an unlocated failure'
        return None, None

    def _target_proposal(self, state):
        """At a start with no certified target-length section yet."""
        cap = self.config.max_probe_samples
        need = self.target_samples(state.start)
        if need is None or need > state.ceiling(cap) or state.certified:
            return None
        if len(state.probes) >= self.config.max_probes_per_start:
            return None
        if not state.probes:
            first = need
            if self.config.first_probe_s is not None:
                wanted = self.timeline.samples_for(state.start, self.config.first_probe_s)
                first = max(need, min(wanted or state.available, state.ceiling(cap)))
            return first, 'first probe at this start'
        # Shrink the shortest failure toward the target, never below it.
        failed = min(state.probes, key=lambda p: p['samples'])
        if failed['samples'] <= need or failed['resource_limited']:
            return None
        return self._shrink(state, failed, need - 1)

    def _extend_proposal(self, state, beat):
        """A longer length at a start, strictly longer than beat samples.

        beat is what the start must exceed: its own certified length when it
        holds one, else the length that would not beat the best section.
        """
        cap = self.config.max_probe_samples
        ceiling = state.ceiling(cap)
        if ceiling <= beat or len(state.probes) >= self.config.max_probes_per_start:
            return None
        failures = [p for p in state.failures_above(beat) if not p['resource_limited']]
        if failures:
            failed = failures[0]
            if self.resolution_samples_between(state.start, beat, failed['samples']):
                return None
            length, why = self._shrink(state, failed, beat)
            if length is None:
                return None
            return min(length, ceiling), why
        if any(p['resource_limited'] and p['samples'] > beat for p in state.probes):
            return None
        if state.certified and beat == state.certified:
            grown = max(beat + 1, int(math.ceil(beat * float(self.config.growth))))
            length = min(grown, ceiling)
            why = 'grow a certified section'
        else:
            length = beat + 1
            why = 'the shortest length that could beat the best section'
        if state.probed(length):
            return None
        return length, why

    def beat_samples(self, start):
        """Samples a section at start needs to beat the best by the resolution, less one."""
        best = self.best
        wanted = self.timeline.duration(*best) + float(self.config.length_resolution_s)
        needed = self.timeline.samples_for(start, wanted)
        return None if needed is None else needed - 1

    def propose(self):
        """(start, samples, phase, reason) of the next probe, or None."""
        if not self.target_met():
            for start, _, _ in self.order:
                proposal = self._target_proposal(self.starts[start])
                if proposal and proposal[0] is not None:
                    return start, proposal[0], PHASE_TARGET, proposal[1]
            return None
        best_start, _ = self.best
        # Certified starts, longest first, then untried starts, then starts
        # whose failures bound nothing, each in coverage order.
        certified = sorted((s for s in self.starts.values() if s.certified),
                           key=lambda s: (-self.timeline.duration(s.start, s.certified),
                                          s.start != best_start, s.start))
        tiers = [certified,
                 [self.starts[s] for s, _, _ in self.order if not self.starts[s].probes],
                 [self.starts[s] for s, _, _ in self.order
                  if self.starts[s].probes and not self.starts[s].certified]]
        for tier, states in enumerate(tiers):
            for state in states:
                if tier == 0:
                    beat = state.certified
                    if state.start != best_start:
                        needed = self.beat_samples(state.start)
                        if needed is None:
                            continue
                        beat = max(beat, needed)
                else:
                    beat = self.beat_samples(state.start)
                    if beat is None:
                        continue
                proposal = self._extend_proposal(state, beat)
                if proposal:
                    return state.start, proposal[0], PHASE_EXTEND, proposal[1]
        return None

    # ---- budget ----------------------------------------------------------

    def consumed(self):
        return {'probes': len(self.probes),
                'probe_samples': int(sum(p['samples'] for p in self.probes)),
                'probe_hours': float(sum(p.get('seconds', 0.0) for p in self.probes)) / 3600.0}

    def budget_refusal(self, samples):
        budget, used = self.config.budget, self.consumed()
        if budget.probes is not None and used['probes'] + 1 > budget.probes:
            return 'budget_probes'
        if (budget.probe_samples is not None
                and used['probe_samples'] + int(samples) > budget.probe_samples):
            return 'budget_probe_samples'
        if budget.probe_hours is not None and used['probe_hours'] >= budget.probe_hours:
            return 'budget_probe_hours'
        return None

    # ---- driving ---------------------------------------------------------

    def record(self, result):
        state = self.starts.setdefault(result['start'], StartState(
            result['start'], len(self.timeline) - result['start']))
        state.probes.append(result)
        self.probes.append(result)

    def run(self, certify, on_probe=None):
        """Probe until nothing is left to propose or the budget refuses.

        certify(start, samples, phase, reason) returns an outcome(). on_probe,
        when given, is called with each outcome as it is recorded.
        """
        if self.last_start is None:
            self.stop_reason = 'recording_shorter_than_target'
            return self
        if self.target_samples(0) is not None and all(
                (self.target_samples(s) or math.inf) > self.config.max_probe_samples
                for s, _, _ in self.order):
            self.stop_reason = 'target_exceeds_max_probe_samples'
            return self
        while True:
            proposal = self.propose()
            if proposal is None:
                self.stop_reason = 'no_probe_left_under_the_rules'
                return self
            start, samples, phase, reason = proposal
            refusal = self.budget_refusal(samples)
            if refusal:
                self.stop_reason = refusal
                self.next_planned = {'start': start, 'samples': samples, 'phase': phase,
                                     'reason': reason}
                return self
            self.decisions.append({'start': start, 'samples': samples, 'phase': phase,
                                   'reason': reason})
            result = certify(start, samples, phase, reason)
            result = dict(result, phase=phase, reason=reason)
            if result['start'] != start or result['samples'] != samples:
                raise ValueError('the certifier answered a different slice than asked')
            self.record(result)
            if on_probe is not None:
                on_probe(result)

    # ---- what was and was not covered -------------------------------------

    def boundary(self, start, samples):
        """What the probes say about extending the section [start, start+samples)."""
        state = self.starts[start]
        end = start + samples
        if end >= len(self.timeline):
            return {'kind': 'end_of_recording',
                    'statement': 'the section ends at the last recorded sample'}
        bound = state.graph_bound
        if samples >= self.config.max_probe_samples and not state.failures_above(samples):
            return {'kind': 'max_probe_samples', 'next_sample_fails': None,
                    'statement': 'the section reached the per-probe sample cap; nothing '
                                 'longer was probed'}
        if bound is not None and bound <= samples:
            evidence = min((p for p in state.probes if p['graph_bound_samples'] == bound),
                           key=lambda p: p['samples'])
            return {'kind': 'graph_bound', 'next_sample_fails': True,
                    'evidence_probe': probe_key(evidence),
                    'statement': 'the graph from this start disconnects before the next '
                                 f'sample (probe of {evidence["samples"]} samples covered '
                                 f'{bound})'}
        failures = state.failures_above(samples)
        if failures:
            nearest = failures[0]
            gap = nearest['samples'] - samples
            record = {'kind': 'next_sample_failed' if gap == 1 else 'longer_probe_failed',
                      'next_sample_fails': gap == 1,
                      'nearest_failed_samples': nearest['samples'],
                      'unprobed_samples_between': gap - 1,
                      'unprobed_seconds_between': self.timeline.duration(
                          start, nearest['samples']) - self.timeline.duration(start, samples),
                      'failure': nearest['classification'],
                      'resource_limited': nearest['resource_limited'],
                      'evidence_probe': probe_key(nearest)}
            record['statement'] = (
                'the slice one sample longer was probed and failed'
                if gap == 1 else
                f'the nearest longer probe ({gap} samples longer) failed; the '
                f'{gap - 1} lengths between were not probed, and a failure bounds '
                'nothing longer unless it is a graph disconnect')
            return record
        return {'kind': 'not_examined', 'next_sample_fails': None,
                'statement': 'no longer slice from this start was probed; the section '
                             'may extend further'}

    def coverage(self):
        """Per start and overall, what the finite search examined."""
        cap = self.config.max_probe_samples
        best = self.best
        rows = []
        for start, level, hinted in self.order:
            state = self.starts[start]
            row = {'start': start, 'level': level, 'hinted': hinted,
                   'start_time_s': float(self.timeline.times[start]),
                   'probes': [{'samples': p['samples'], 'passed': p['passed'],
                               'class': p['classification']['class'],
                               'graph_bound_samples': p['graph_bound_samples'],
                               'resource_limited': p['resource_limited']}
                              for p in state.probes],
                   'certified_samples': state.certified or None,
                   'graph_bound_samples': state.graph_bound}
            need = self.target_samples(start)
            if state.certified:
                row['status'] = 'certified'
            elif need is None or need > min(state.available, cap):
                row['status'] = 'target_longer_than_max_probe_samples'
            elif state.graph_bound is not None and state.graph_bound < need:
                row['status'] = 'bounded_below_target_by_graph'
            elif state.probes:
                row['status'] = 'failed_probes_bound_nothing'
            else:
                row['status'] = 'not_probed'
            if best is not None and not state.certified:
                beat = self.beat_samples(start)
                row['could_beat_best_within_cap'] = bool(
                    beat is not None and state.ceiling(cap) > beat)
            rows.append(row)
        probed = [r['start'] for r in rows if r['probes']]
        feasible_span = (0.0 if self.last_start is None
                         else float(self.timeline.times[self.last_start] - self.timeline.times[0]))
        edges = sorted(set(probed))
        largest_gap = None
        if self.last_start is not None:
            points = [0] + edges + [self.last_start]
            largest_gap = max(float(self.timeline.times[b] - self.timeline.times[a])
                              for a, b in zip(points, points[1:])) if len(points) > 1 else 0.0
            if not edges:
                largest_gap = feasible_span
        counts = {}
        for row in rows:
            counts[row['status']] = counts.get(row['status'], 0) + 1
        return {
            'grid_starts': len(rows),
            'grid_levels': [len(level) for level in self.levels],
            'stride_samples': self.stride_samples,
            'feasible_start_span_s': feasible_span,
            'probed_starts': len(probed),
            'status_counts': counts,
            'largest_unprobed_start_gap_s': largest_gap,
            'starts': rows,
            'statements': self.statements(rows),
        }

    def statements(self, rows):
        cap = self.config.max_probe_samples
        said = [
            f'starts were drawn from a coverage grid with a stride of '
            f'{self.config.start_stride_s:g} s ({self.stride_samples} samples); '
            f'{sum(1 for r in rows if r["probes"])} of {len(rows)} grid starts were '
            'probed. Starts between grid points were never probed',
            f'no slice longer than {cap} samples was probed (the per-probe resource cap), '
            'so no section longer than that can have been found',
            'a failed probe is evidence about its own slice only, except a graph '
            'disconnect, which bounds that one start; no start was excluded because '
            'another start failed or because a heuristic ranked it low',
            f'lengths were resolved to {self.config.length_resolution_s:g} s; the longest '
            'certified section is the longest found by this search under its budget, not '
            'a proof that nothing longer is executable',
        ]
        unprobed = [r['start'] for r in rows if r['status'] == 'not_probed']
        if unprobed:
            said.append(f'{len(unprobed)} grid starts were never probed')
        open_failures = [r['start'] for r in rows
                         if r['status'] == 'failed_probes_bound_nothing']
        if open_failures:
            said.append(f'{len(open_failures)} starts failed only in ways that bound '
                        'nothing; other lengths there were not all probed')
        return said

    def summary(self):
        best = self.best
        return {
            'config': self.config.as_dict(),
            'stop_reason': self.stop_reason,
            'next_planned_probe': self.next_planned,
            'budget_consumed': self.consumed(),
            'target_met': self.target_met(),
            'best': None if best is None else dict(
                self.timeline.interval(*best), boundary=self.boundary(*best)),
            'certified_sections': [
                dict(self.timeline.interval(s.start, s.certified),
                     boundary=self.boundary(s.start, s.certified))
                for s in sorted(self.starts.values(), key=lambda s: s.start) if s.certified],
            'decisions': list(self.decisions),
        }


def probe_key(probe):
    return {'start': probe['start'], 'samples': probe['samples']}
