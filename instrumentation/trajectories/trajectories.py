"""trajectories.py — primitive flight profiles for the test harness.

Each generator produces a sequence of (time, position, velocity, attitude)
samples at a configured rate. Used by the test harness to drive the
simulated drone through standardized flight scenarios so per-subsystem
load can be measured under known motion.

Four standard scenarios are defined, in increasing intensity:

  1. cruise_straight    — constant heading, modest speed (workload floor)
  2. gentle_maneuver    — slow yaw + altitude changes (typical search)
  3. medium_maneuver    — banking turns up to 30° (typical pursuit)
  4. aerobatic_forest   — rapid yaw/roll/pitch + altitude (worst case)

Why these four: they cover the workload envelope from "perception runs at
floor cost while VIO sees almost no optical flow" through "perception and
VIO under simultaneous stress with rapid attitude changes." The chip
spec needs to handle scenario 4; the chip's energy budget is dominated
by scenarios 1-2.

Output convention: each sample is a TrajectorySample with monotonic time
in seconds. Position is NED meters from start, velocity is body-frame
m/s, attitude is roll/pitch/yaw radians. Optical flow magnitude is
included as a derived quantity because it's the headline workload driver
for VIO and tracking.
"""

from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Iterator, List


@dataclass(frozen=True)
class TrajectorySample:
    """One tick of trajectory state."""
    t_s: float                  # seconds since trajectory start
    pos_n: float                # north position (m)
    pos_e: float                # east position (m)
    pos_d: float                # down position (m, negative is up)
    vel_x: float                # body-frame forward velocity (m/s)
    vel_y: float                # body-frame right velocity (m/s)
    vel_z: float                # body-frame down velocity (m/s)
    roll: float                 # rad
    pitch: float                # rad
    yaw: float                  # rad
    yaw_rate: float             # rad/s
    optical_flow_norm: float    # 0..1 normalized magnitude of expected optical flow

    @property
    def alt_m(self) -> float:
        return -self.pos_d


# ──────────────────────────────────────────────────────────────────────
# Profile generators
# ──────────────────────────────────────────────────────────────────────

def cruise_straight(
    duration_s: float = 60.0,
    speed_mps: float = 8.0,
    altitude_m: float = 50.0,
    heading_rad: float = 0.0,
    rate_hz: float = 50.0,
) -> Iterator[TrajectorySample]:
    """Scenario 1: straight, level cruise.

    Constant heading, constant altitude, constant velocity. The cheapest
    workload — perception at floor cost, VIO sees uniform translation,
    encode at steady bitrate. Use as the baseline against which other
    scenarios are compared.
    """
    dt = 1.0 / rate_hz
    n = int(duration_s * rate_hz)
    cos_h, sin_h = math.cos(heading_rad), math.sin(heading_rad)
    for i in range(n):
        t = i * dt
        pos_n = speed_mps * t * cos_h
        pos_e = speed_mps * t * sin_h
        yield TrajectorySample(
            t_s=t,
            pos_n=pos_n, pos_e=pos_e, pos_d=-altitude_m,
            vel_x=speed_mps, vel_y=0.0, vel_z=0.0,
            roll=0.0, pitch=0.0, yaw=heading_rad,
            yaw_rate=0.0,
            optical_flow_norm=0.2,    # just translation, low rotational flow
        )


def gentle_maneuver(
    duration_s: float = 60.0,
    speed_mps: float = 8.0,
    altitude_m: float = 50.0,
    yaw_amplitude_rad: float = math.radians(20),
    altitude_amplitude_m: float = 5.0,
    period_s: float = 20.0,
    rate_hz: float = 50.0,
) -> Iterator[TrajectorySample]:
    """Scenario 2: gentle yawing + altitude oscillation.

    Sinusoidal yaw and altitude variation, no banking. Models a typical
    search pattern where the drone is gently weaving and adjusting
    height to scan an area. Perception load steady, VIO sees moderate
    rotational flow, FSM transitions exercised but not stressed.
    """
    dt = 1.0 / rate_hz
    omega = 2 * math.pi / period_s
    n = int(duration_s * rate_hz)
    pos_n = pos_e = 0.0
    for i in range(n):
        t = i * dt
        yaw = yaw_amplitude_rad * math.sin(omega * t)
        yaw_rate = yaw_amplitude_rad * omega * math.cos(omega * t)
        # Altitude oscillation 90° out of phase with yaw
        alt = altitude_m + altitude_amplitude_m * math.sin(omega * t + math.pi / 2)
        # Integrate position from velocity in current heading
        vel_n = speed_mps * math.cos(yaw)
        vel_e = speed_mps * math.sin(yaw)
        pos_n += vel_n * dt
        pos_e += vel_e * dt
        # Optical flow magnitude scales roughly with yaw rate
        flow = 0.2 + 0.5 * abs(yaw_rate) / max(yaw_amplitude_rad * omega, 1e-6)
        yield TrajectorySample(
            t_s=t,
            pos_n=pos_n, pos_e=pos_e, pos_d=-alt,
            vel_x=speed_mps, vel_y=0.0,
            vel_z=altitude_amplitude_m * omega * math.cos(omega * t + math.pi / 2),
            roll=0.0, pitch=0.0, yaw=yaw, yaw_rate=yaw_rate,
            optical_flow_norm=min(1.0, flow),
        )


def medium_maneuver(
    duration_s: float = 60.0,
    speed_mps: float = 12.0,
    altitude_m: float = 30.0,
    bank_angle_rad: float = math.radians(30),
    turn_radius_m: float = 25.0,
    rate_hz: float = 50.0,
) -> Iterator[TrajectorySample]:
    """Scenario 3: figure-eight at 30° bank.

    Continuous banking turns alternating left and right. Sustained
    rotational + translational flow, exercises VIO under realistic
    pursuit conditions, perception model needs to track moving features
    through banking attitude. This is the "drone is following something"
    case.
    """
    dt = 1.0 / rate_hz
    n = int(duration_s * rate_hz)
    # Figure-eight: yaw rate alternates sign every half lap.
    # Turn period = 2π × radius / speed for one circle; figure-eight is two of these.
    circle_period = 2 * math.pi * turn_radius_m / speed_mps
    pos_n = pos_e = 0.0
    yaw = 0.0
    for i in range(n):
        t = i * dt
        # Direction of yaw rate flips every half circle
        half = math.floor(2 * t / circle_period) % 2
        sign = 1.0 if half == 0 else -1.0
        yaw_rate = sign * speed_mps / turn_radius_m
        yaw += yaw_rate * dt
        roll = sign * bank_angle_rad
        vel_n = speed_mps * math.cos(yaw)
        vel_e = speed_mps * math.sin(yaw)
        pos_n += vel_n * dt
        pos_e += vel_e * dt
        # Higher optical flow due to banking + yaw together
        yield TrajectorySample(
            t_s=t,
            pos_n=pos_n, pos_e=pos_e, pos_d=-altitude_m,
            vel_x=speed_mps, vel_y=0.0, vel_z=0.0,
            roll=roll, pitch=0.0, yaw=yaw, yaw_rate=yaw_rate,
            optical_flow_norm=0.7,
        )


def aerobatic_forest(
    duration_s: float = 60.0,
    speed_mps: float = 10.0,
    altitude_m: float = 15.0,
    rate_hz: float = 50.0,
    seed: int = 42,
) -> Iterator[TrajectorySample]:
    """Scenario 4: rapid 3-axis maneuvering through simulated forest gaps.

    Pseudo-random rapid yaw + roll + pitch + altitude changes. Models
    pilot dodging branches at speed. Worst-case workload: perception
    must track through rapid attitude changes, VIO sees high rotational
    flow with translation, encode produces high-bitrate output due to
    scene change, glass-to-glass latency budget is binding.

    Deterministic for a given seed so tests are reproducible.
    """
    dt = 1.0 / rate_hz
    n = int(duration_s * rate_hz)
    # Use a deterministic noise generator that doesn't pull in numpy.
    # We use a simple sum-of-sines with prime frequencies for "random-looking"
    # but reproducible motion.
    def osc(t: float, freqs: list[float], phases: list[float]) -> float:
        return sum(math.sin(2 * math.pi * f * t + p)
                   for f, p in zip(freqs, phases)) / len(freqs)

    yaw_freqs = [0.7, 1.3, 2.1]
    roll_freqs = [1.1, 1.9, 3.1]
    pitch_freqs = [0.9, 1.7, 2.5]
    alt_freqs = [0.3, 0.5]

    # Seed-derived phase offsets
    rng = (seed * 0x9E3779B97F4A7C15) & ((1 << 64) - 1)
    def next_phase() -> float:
        nonlocal rng
        rng = (rng * 6364136223846793005 + 1442695040888963407) & ((1 << 64) - 1)
        return (rng / (1 << 64)) * 2 * math.pi

    yaw_phases = [next_phase() for _ in yaw_freqs]
    roll_phases = [next_phase() for _ in roll_freqs]
    pitch_phases = [next_phase() for _ in pitch_freqs]
    alt_phases = [next_phase() for _ in alt_freqs]

    pos_n = pos_e = 0.0
    yaw = 0.0
    prev_yaw = 0.0
    for i in range(n):
        t = i * dt
        yaw_rate_target = osc(t, yaw_freqs, yaw_phases) * 1.5     # rad/s
        roll = osc(t, roll_freqs, roll_phases) * math.radians(45)
        pitch = osc(t, pitch_freqs, pitch_phases) * math.radians(20)
        alt_offset = osc(t, alt_freqs, alt_phases) * 5.0
        # Integrate yaw
        yaw += yaw_rate_target * dt
        yaw_rate = (yaw - prev_yaw) / dt
        prev_yaw = yaw
        # Position integration (slowed when banking hard — reasonable approx)
        speed_eff = speed_mps * max(0.3, math.cos(roll))
        vel_n = speed_eff * math.cos(yaw)
        vel_e = speed_eff * math.sin(yaw)
        pos_n += vel_n * dt
        pos_e += vel_e * dt
        # Optical flow is high — high yaw + roll + pitch combined
        yield TrajectorySample(
            t_s=t,
            pos_n=pos_n, pos_e=pos_e, pos_d=-(altitude_m + alt_offset),
            vel_x=speed_eff, vel_y=0.0, vel_z=0.0,
            roll=roll, pitch=pitch, yaw=yaw, yaw_rate=yaw_rate,
            optical_flow_norm=min(1.0, 0.8 + 0.2 * abs(yaw_rate) / 3.0),
        )


# ──────────────────────────────────────────────────────────────────────
# Convenience: by-name dispatch
# ──────────────────────────────────────────────────────────────────────

PROFILES = {
    "cruise_straight":  cruise_straight,
    "gentle_maneuver":  gentle_maneuver,
    "medium_maneuver":  medium_maneuver,
    "aerobatic_forest": aerobatic_forest,
}


def generate(profile: str, **kwargs) -> List[TrajectorySample]:
    """Return a list of samples for the named profile.

    Eagerly materializes — these are short trajectories (seconds × hz =
    a few thousand samples). For long-running streaming use the
    generator functions directly.
    """
    if profile not in PROFILES:
        raise ValueError(f"Unknown profile: {profile}. Choose from {list(PROFILES)}")
    return list(PROFILES[profile](**kwargs))
