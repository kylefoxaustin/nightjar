"""Unit tests for trajectory primitives."""

import math
import pytest

from instrumentation.trajectories import (
    TrajectorySample, PROFILES, generate,
    cruise_straight, gentle_maneuver, medium_maneuver, aerobatic_forest,
)


class TestTrajectoryShape:
    """All profiles should produce well-formed samples."""

    @pytest.mark.parametrize("name", list(PROFILES.keys()))
    def test_profile_produces_samples(self, name):
        samples = generate(name, duration_s=2.0, rate_hz=50.0)
        assert len(samples) == 100, f"{name}: expected 100 samples (2s × 50Hz), got {len(samples)}"

    @pytest.mark.parametrize("name", list(PROFILES.keys()))
    def test_time_monotonic(self, name):
        samples = generate(name, duration_s=2.0, rate_hz=50.0)
        for a, b in zip(samples, samples[1:]):
            assert b.t_s > a.t_s

    @pytest.mark.parametrize("name", list(PROFILES.keys()))
    def test_optical_flow_in_unit_range(self, name):
        samples = generate(name, duration_s=2.0, rate_hz=50.0)
        for s in samples:
            assert 0.0 <= s.optical_flow_norm <= 1.0


class TestCruiseStraight:
    def test_constant_heading(self):
        samples = generate("cruise_straight", duration_s=10.0, rate_hz=10.0)
        # Yaw should be constant at heading_rad (default 0)
        assert all(s.yaw == 0.0 for s in samples)

    def test_constant_altitude(self):
        samples = generate("cruise_straight", duration_s=10.0, rate_hz=10.0,
                           altitude_m=50.0)
        assert all(abs(s.alt_m - 50.0) < 1e-6 for s in samples)

    def test_low_optical_flow(self):
        # Pure translation should produce low optical flow
        samples = generate("cruise_straight", duration_s=10.0, rate_hz=10.0)
        avg_flow = sum(s.optical_flow_norm for s in samples) / len(samples)
        assert avg_flow < 0.3


class TestAerobaticForest:
    def test_deterministic_with_seed(self):
        s1 = generate("aerobatic_forest", duration_s=5.0, rate_hz=20.0, seed=42)
        s2 = generate("aerobatic_forest", duration_s=5.0, rate_hz=20.0, seed=42)
        for a, b in zip(s1, s2):
            assert a.yaw == b.yaw
            assert a.roll == b.roll

    def test_different_seeds_produce_different_trajectories(self):
        s1 = generate("aerobatic_forest", duration_s=5.0, rate_hz=20.0, seed=42)
        s2 = generate("aerobatic_forest", duration_s=5.0, rate_hz=20.0, seed=99)
        # At least some samples should differ
        diffs = sum(1 for a, b in zip(s1, s2) if abs(a.yaw - b.yaw) > 0.01)
        assert diffs > 10

    def test_high_optical_flow(self):
        # Aerobatic should produce high optical flow
        samples = generate("aerobatic_forest", duration_s=5.0, rate_hz=20.0)
        avg_flow = sum(s.optical_flow_norm for s in samples) / len(samples)
        assert avg_flow > 0.7


class TestMediumManeuver:
    def test_banking_present(self):
        samples = generate("medium_maneuver", duration_s=10.0, rate_hz=20.0)
        # Should have non-zero roll throughout
        rolls = [abs(s.roll) for s in samples]
        assert max(rolls) > math.radians(10)


class TestGentleManeuver:
    def test_yaw_oscillates(self):
        samples = generate("gentle_maneuver", duration_s=20.0, rate_hz=20.0,
                           period_s=10.0, yaw_amplitude_rad=math.radians(30))
        yaws = [s.yaw for s in samples]
        # Should oscillate — both positive and negative values
        assert max(yaws) > 0.1
        assert min(yaws) < -0.1


class TestUnknownProfileRaises:
    def test_unknown_profile_raises(self):
        with pytest.raises(ValueError):
            generate("not_a_real_profile", duration_s=1.0)
