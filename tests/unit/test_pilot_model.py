"""Unit tests for the latency-aware pilot model."""

import pytest

from instrumentation.pilots import (
    observe, aggregate, PilotObservation,
    PILOT_VISUAL_REACTION_MS,
    FLYABILITY_THRESHOLD_GOOD_MS,
    FLYABILITY_THRESHOLD_DEGRADED_MS,
)


class TestObserve:
    def test_zero_latency_full_quality(self):
        obs = observe(g2g_latency_ms=0.0, drone_speed_mps=8.0)
        assert obs.control_quality_pct == 100.0
        assert obs.flyability_status == "good"

    def test_at_good_threshold_still_flyable(self):
        obs = observe(g2g_latency_ms=FLYABILITY_THRESHOLD_GOOD_MS, drone_speed_mps=8.0)
        assert obs.flyability_status == "good"

    def test_above_good_threshold_degraded(self):
        obs = observe(g2g_latency_ms=120.0, drone_speed_mps=8.0)
        assert obs.flyability_status == "degraded"

    def test_above_degraded_threshold_unflyable(self):
        obs = observe(g2g_latency_ms=200.0, drone_speed_mps=8.0)
        assert obs.flyability_status == "unflyable"
        assert not obs.flyable

    def test_overshoot_scales_with_speed(self):
        slow = observe(g2g_latency_ms=100.0, drone_speed_mps=5.0)
        fast = observe(g2g_latency_ms=100.0, drone_speed_mps=15.0)
        # Higher speed at same latency should produce 3× the overshoot
        assert fast.estimated_overshoot_m == pytest.approx(3.0 * slow.estimated_overshoot_m, rel=0.01)

    def test_overshoot_scales_with_latency(self):
        low = observe(g2g_latency_ms=50.0, drone_speed_mps=8.0)
        high = observe(g2g_latency_ms=500.0, drone_speed_mps=8.0)
        # Higher latency means more overshoot
        assert high.estimated_overshoot_m > low.estimated_overshoot_m * 2

    def test_effective_reaction_includes_visual_floor(self):
        obs = observe(g2g_latency_ms=50.0, drone_speed_mps=8.0)
        assert obs.effective_reaction_ms == 50.0 + PILOT_VISUAL_REACTION_MS

    def test_quality_falls_monotonically(self):
        latencies = [0, 50, 100, 150, 200, 300, 500]
        qualities = [observe(g2g_latency_ms=l, drone_speed_mps=8.0).control_quality_pct
                     for l in latencies]
        for a, b in zip(qualities, qualities[1:]):
            assert a >= b   # never increases


class TestAggregate:
    def test_empty(self):
        s = aggregate([])
        assert s["n"] == 0
        assert s["flyable"] is True

    def test_all_good(self):
        obs = [observe(50.0, 8.0) for _ in range(100)]
        s = aggregate(obs)
        assert s["n"] == 100
        assert s["mission_flyable"] is True
        assert s["pct_unflyable"] == 0.0
        assert s["pct_degraded"] == 0.0

    def test_one_unflyable_breaks_mission(self):
        obs = [observe(50.0, 8.0) for _ in range(99)] + [observe(200.0, 8.0)]
        s = aggregate(obs)
        assert s["mission_flyable"] is False
        assert s["pct_unflyable"] == 1.0

    def test_p99_overshoot_reflects_worst_cases(self):
        # 99 fast samples + 1 very slow
        obs = [observe(50.0, 8.0) for _ in range(99)] + [observe(190.0, 12.0)]
        s = aggregate(obs)
        # p99 should be much higher than the median (which is dominated by the 50ms cases)
        assert s["p99_overshoot_m"] > 1.0
