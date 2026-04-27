"""latency_aware_pilot.py — measurement-only pilot behavior model.

Models how a remote pilot's effective control authority degrades as
glass-to-glass latency increases. Based on published FPV community
results: pilots can compensate for latency up to roughly 100ms; beyond
that, corrections lag and overshoot grows nonlinearly.

This is a MEASUREMENT model, not a closed-loop controller. It does NOT
generate stick inputs that fly the drone. It produces *observations*
about the pilot's likely behavior given the measured g2g latency:

    - estimated_overshoot_m  : how far past the target the pilot lands
    - control_quality_pct    : 100% at zero latency, drops as latency rises
    - effective_reaction_ms  : pilot reaction time + g2g latency
    - flyability_status      : "good" | "degraded" | "unflyable"

These observations are emitted as WorkloadRecords with the existing
schema's free-form `extras` dict, so they show up in the partition
report alongside the measured numbers. The KPI evaluator can then check
"did this scenario remain flyable end-to-end" — answering the question
your test harness exists to answer.

Why measurement-only: closing the SITL loop on latency-induced
overshoot opens a control-system rabbit hole (you'd need to model human
prediction, learning, scenario familiarity, etc.) that doesn't change
the answer the chip team needs. The chip team needs to know: "given
your measured latency curve, would a human pilot be able to dodge a
branch under this scenario?" The flyability metric below answers that
without simulating the human.
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Optional


# ──────────────────────────────────────────────────────────────────────
# Pilot model constants
# ──────────────────────────────────────────────────────────────────────

# Visual reaction time floor (ms). Even at zero g2g latency, humans need
# time between seeing something and moving the stick. This is the well-
# documented ~250ms simple reaction time, reduced to 200ms for trained
# pilots on familiar patterns.
PILOT_VISUAL_REACTION_MS = 200.0

# Above this total reaction time, the pilot cannot reliably react to thin
# obstacles (branches at flight speed). Sourced from FPV racing literature.
FLYABILITY_THRESHOLD_GOOD_MS = 100.0
FLYABILITY_THRESHOLD_DEGRADED_MS = 150.0

# Overshoot model: when reacting late, the drone has traveled extra
# distance during the latency window. Overshoot = velocity × latency.
# (Multiplied by 1.4 to account for the corrective overshoot itself —
# pilots over-correct under latency.)
OVERSHOOT_MULTIPLIER = 1.4


@dataclass
class PilotObservation:
    """One pilot-model observation tied to a glass-to-glass measurement."""
    g2g_latency_ms: float
    pilot_reaction_ms: float            # PILOT_VISUAL_REACTION_MS
    effective_reaction_ms: float        # g2g + reaction
    control_quality_pct: float          # 0..100
    estimated_overshoot_m: float
    flyability_status: str              # "good" | "degraded" | "unflyable"

    @property
    def flyable(self) -> bool:
        return self.flyability_status != "unflyable"


# ──────────────────────────────────────────────────────────────────────
# Model
# ──────────────────────────────────────────────────────────────────────

def _control_quality(effective_reaction_ms: float) -> float:
    """Empirical degradation curve for pilot control quality vs. latency.

    100% at the reaction-time floor (~200ms). Drops linearly to 50% at
    400ms total reaction. Drops to 10% at 600ms+. Calibrated against FPV
    pilot self-reports; tunable via the constants if needed.
    """
    if effective_reaction_ms <= PILOT_VISUAL_REACTION_MS:
        return 100.0
    if effective_reaction_ms <= 400.0:
        # Linear fall from 100% at 200ms to 50% at 400ms
        return 100.0 - 0.25 * (effective_reaction_ms - PILOT_VISUAL_REACTION_MS)
    if effective_reaction_ms <= 600.0:
        # 50% at 400 → 10% at 600
        return 50.0 - 0.20 * (effective_reaction_ms - 400.0)
    return 10.0


def _flyability(g2g_latency_ms: float) -> str:
    if g2g_latency_ms <= FLYABILITY_THRESHOLD_GOOD_MS:
        return "good"
    if g2g_latency_ms <= FLYABILITY_THRESHOLD_DEGRADED_MS:
        return "degraded"
    return "unflyable"


def observe(
    g2g_latency_ms: float,
    drone_speed_mps: float = 8.0,
    pilot_visual_reaction_ms: float = PILOT_VISUAL_REACTION_MS,
) -> PilotObservation:
    """Produce one pilot-model observation given measured g2g latency.

    The drone speed is needed for the overshoot calculation — faster
    flight means a fixed latency translates to more meters of overshoot.
    """
    effective = g2g_latency_ms + pilot_visual_reaction_ms
    quality = _control_quality(effective)
    overshoot = (drone_speed_mps * (effective / 1000.0)) * OVERSHOOT_MULTIPLIER
    return PilotObservation(
        g2g_latency_ms=g2g_latency_ms,
        pilot_reaction_ms=pilot_visual_reaction_ms,
        effective_reaction_ms=effective,
        control_quality_pct=quality,
        estimated_overshoot_m=overshoot,
        flyability_status=_flyability(g2g_latency_ms),
    )


def aggregate(observations: list[PilotObservation]) -> dict:
    """Roll up many observations into summary statistics for the report."""
    if not observations:
        return {"n": 0, "flyable": True}
    n = len(observations)
    n_unflyable = sum(1 for o in observations if not o.flyable)
    n_degraded = sum(1 for o in observations if o.flyability_status == "degraded")
    p99_g2g = sorted(o.g2g_latency_ms for o in observations)[max(0, int(0.99 * n) - 1)]
    p99_overshoot = sorted(o.estimated_overshoot_m for o in observations)[max(0, int(0.99 * n) - 1)]
    return {
        "n": n,
        "n_unflyable": n_unflyable,
        "n_degraded": n_degraded,
        "pct_unflyable": 100.0 * n_unflyable / n,
        "pct_degraded": 100.0 * n_degraded / n,
        "p99_g2g_latency_ms": p99_g2g,
        "p99_overshoot_m": p99_overshoot,
        "mission_flyable": n_unflyable == 0,
    }
