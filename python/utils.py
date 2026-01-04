import math
import numpy as np

def normalize_angle(a: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return (a + math.pi) % (2*math.pi) - math.pi


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def safe_inverse_3x3(A: np.ndarray) -> np.ndarray:
    """Attempt to invert a 3x3 matrix; raise if singular."""
    return np.linalg.inv(A)
