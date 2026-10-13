"""Path shortcutting (exercise 3)."""
import numpy as np

from .collision import is_segment_free
from .path import path_length, point_at


def shortcut(env, path, maxrep):
    """Returns a new path; the input is not changed."""
    path = [tuple(p) for p in path]
    for _ in range(maxrep):
        L = path_length(path)
        t1, t2 = sorted(np.random.rand(2) * L)
        k1, q1 = point_at(path, t1)
        k2, q2 = point_at(path, t2)
        if k1 == k2:  # same segment, nothing to gain
            continue
        if is_segment_free(env, q1, q2):
            # keep path[0..k1], then q1 -> q2, then path[k2 + 1..]
            path = path[:k1 + 1] + [tuple(q1), tuple(q2)] + path[k2 + 1:]
    return path
