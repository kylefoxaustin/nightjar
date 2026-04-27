# ADR 011: Trajectory-driven test harness with measurement-only pilot model

**Status:** Accepted
**Date:** 2026-04
**Decision drivers:** test reproducibility, separation of "is the chip
fast enough?" from "can a human fly this?", avoidance of human-in-the-
loop modeling complexity.

## Context

The project needed a way to systematically exercise the pipeline under
known motion profiles. Three structural questions:

1. **What drives the drone's motion in the test?** A real pilot, a
   simulated pilot, a programmatic trajectory, or a closed-loop
   navigation controller?
2. **How is the human in the loop modeled?** Real (impossible in CI),
   recorded (acceptable but limited), simulated (rabbit hole), or
   measurement-only (loses some realism but gains tractability)?
3. **How many scenarios?** One scenario isn't enough; sweeping every
   possible flight pattern is too many.

## Decision

**Trajectory-driven, four scenarios, measurement-only pilot model.**

- Motion is driven by **trajectory generators** (`instrumentation/
  trajectories/trajectories.py`). Four standard profiles:
  `cruise_straight`, `gentle_maneuver`, `medium_maneuver`,
  `aerobatic_forest`. Each is pure math, deterministic given seed,
  produces (time, position, velocity, attitude, optical_flow_norm)
  tuples at a configured rate.

- Pilot is modeled as **measurement-only** via
  `instrumentation/pilots/latency_aware_pilot.py`. Given the measured
  glass-to-glass latency, it outputs flyability observations
  (control_quality_pct, estimated_overshoot_m, flyability_status:
  good/degraded/unflyable). It does NOT generate stick inputs that
  affect the SITL drone's motion.

- The pilot model lives in a separate ROS 2 node (`drone_pilot`) so
  future closed-loop pilot variants can plug in via the same interface.

## Alternatives considered

- **Real human pilot in the loop.** Rejected: not reproducible, can't
  run in CI, and the human is exactly what we're trying to abstract.

- **Recorded human stick replay.** Considered. Useful but requires
  non-trivial capture infrastructure (instrumented FPV setup,
  representative pilots, cleaning, time-aligning recordings to camera
  feeds). Hooked for future support but not implemented in v0.2.0.

- **Programmatic closed-loop pilot.** Rejected for now: modeling
  predictive human control under varying latency is its own research
  problem. The chip team's question is "can a human fly this?", not
  "what does an artificial-pilot policy look like?". We answer the
  former without solving the latter.

- **One mega-scenario.** Rejected: doesn't isolate workload regimes.
  Need separate "floor" and "ceiling" data points to size the chip's
  steady-state vs. peak budget.

- **N>4 scenarios.** Rejected for v0.2.0: four covers the workload
  envelope cleanly. More scenarios can be added later by appending
  to `PROFILES` in `trajectories.py`.

## Why these specific four scenarios

The four cover the workload envelope:

1. **cruise_straight** is the floor. Constant heading, constant
   altitude, low optical flow. Any chip that fails this fails
   everything.

2. **gentle_maneuver** is search-pattern realistic. Moderate yaw
   oscillation + altitude variation. Steady perception load,
   intermittent VIO stress.

3. **medium_maneuver** is figure-eight at 30° bank. Sustained
   rotational + translational flow. Banking attitude exercises
   perception model invariance, VIO sees realistic pursuit motion.

4. **aerobatic_forest** is the ceiling. Pseudo-random rapid 3-axis
   maneuvering. Models pilot dodging branches at speed. Worst case
   for every subsystem simultaneously, and the binding case for
   glass-to-glass latency.

## Pilot model rationale

Latency-aware measurement is the right abstraction because:

- The chip team needs to defend a g2g budget. The pilot model
  translates that budget into an *operationally meaningful* question
  ("at this measured g2g, can a human fly the drone?") rather than
  a raw latency number.

- Empirical FPV pilot data is available. ~200ms visual reaction
  floor, flyability degrades sharply above 100ms total g2g, becomes
  unflyable around 150ms. These thresholds are tunable via the
  module's constants.

- It's a **filter** on the measured latency distribution. The
  partition report can show "p99 g2g = 87ms (PASS), but pilot
  flyable 92% of frames during aerobatic_forest (FAIL — needs
  >99%)". That's a much sharper claim than the raw number alone.

## Consequences

- The test harness is reproducible (deterministic seeds for
  `aerobatic_forest`, pure math for the others). Same input →
  same output across machines and runs.

- Adding a new flight scenario is a one-function-plus-yaml change
  (~30 lines).

- Future closed-loop pilots can plug into the same interface by
  changing `$PILOT_MODEL` env var without touching the rest of the
  stack.

- The KPI evaluator gets a new pass/fail axis (pilot flyability).
  This is the most operationally relevant KPI in the entire model
  for the FPV use case.

## Worth knowing

The pilot constants (200ms visual reaction, 100ms good threshold,
150ms unflyable threshold) are calibrated to FPV racing community
norms. For SAR rescue use cases the operator may be less skilled
and these thresholds should be tighter (consider bumping the visual
reaction to 250ms and the flyable threshold to 80ms). Slider
overrides via `pilot_visual_reaction_ms` and the YAML mission spec
allow per-scenario tuning.
