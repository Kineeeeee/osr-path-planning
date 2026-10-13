"""Helpers for paths given as polylines [p0, p1, ..., pn]."""
import numpy as np

def path_length(path):
    """Total length of the polyline."""
    ans = 0
    for i in range(len(path) - 1):
        ans += np.linalg.norm(np.subtract(path[i + 1], path[i]))
    return ans

def point_at(path, t):
    """Point q(t) at distance t from the start, measured along the path.

    Returns (k, q) where q lies on the segment path[k] -> path[k + 1].
    """
    for k in range(len(path) - 1):
        a, b = np.array(path[k]), np.array(path[k + 1])
        seg = np.linalg.norm(np.subtract(b, a))
        if seg == 0:
            continue
        if t <= seg:
            q = a + (t / seg) * (b - a)
            return k, q
        t -= seg
    return len(path) - 2, np.array(path[-1])  # t slightly above the length (rounding)
