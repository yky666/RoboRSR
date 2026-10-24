"""Simulation-only Aurora Lite opening contract, shared by URDF and MJCF adapters.

Public opening is the physical inner-pad gap in metres. Existing policy actions
remain normalized (0 closed, 1 open). No real-device SDK is imported here.
"""

import math

MAX_OPENING_M = 0.085
CAD_TO_MOUNT_Z_M = 0.07850000009999997
CHANNELS = tuple(f"{arm}_{finger}" for arm in ("left", "right") for finger in ("positive", "negative"))


def checked_opening(opening_m):
    value = float(opening_m)
    if not math.isfinite(value) or not 0.0 <= value <= MAX_OPENING_M:
        raise ValueError("Aurora opening must be finite and in [0, 0.085] metres")
    return value


def urdf_jaw_positions(opening_m):
    """Equal outward travel from the URDF's closed reference."""
    q = checked_opening(opening_m) / 2.0
    return q, q


def mjcf_closure_positions(opening_m):
    """Equal inward travel from the supplier CAD's fully open reference."""
    q = (MAX_OPENING_M - checked_opening(opening_m)) / 2.0
    return q, q


def measured_opening(first_jaw_m, second_jaw_m, *, convention="urdf"):
    values = float(first_jaw_m), float(second_jaw_m)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("Non-finite measured jaw position")
    if convention == "urdf":
        return sum(values)
    if convention == "mjcf":
        return MAX_OPENING_M - sum(values)
    raise ValueError("Unknown jaw convention")


def normalized_action(opening_m):
    return checked_opening(opening_m) / MAX_OPENING_M
