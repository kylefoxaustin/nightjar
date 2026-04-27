"""Pilot behavior models.

Currently includes:
  - latency_aware_pilot: measurement-only model that quantifies pilot
    flyability as a function of measured g2g latency

Future hooks (interface stubs reserved):
  - recorded_pilot:  replays captured stick input from a real pilot
  - rule_based_pilot: simple programmatic pilot for closed-loop SITL
"""
from .latency_aware_pilot import (
    PilotObservation,
    observe,
    aggregate,
    PILOT_VISUAL_REACTION_MS,
    FLYABILITY_THRESHOLD_GOOD_MS,
    FLYABILITY_THRESHOLD_DEGRADED_MS,
)

__all__ = [
    "PilotObservation", "observe", "aggregate",
    "PILOT_VISUAL_REACTION_MS",
    "FLYABILITY_THRESHOLD_GOOD_MS", "FLYABILITY_THRESHOLD_DEGRADED_MS",
]
