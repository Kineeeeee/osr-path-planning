"""Collision check for straight segments."""
import numpy as np
from . import config

def is_segment_free(env, p, q):
    """True if the segment p-q is collision-free, checked as the exercise suggests."""
    return segment_free_sampled(env, p, q, config.COLLISION_STEP)

def segment_free_sampled(env, p, q, step):
    """Call env.check_collision on points along p-q."""
    p, q = np.array(p, dtype=float), np.array(q, dtype=float)
    n = int(np.linalg.norm(q - p) / step) + 1
    for t in np.linspace(0, 1, n + 1):  # t = 0 is p, t = 1 is q
        x, y = p + t * (q - p)
        if env.check_collision(x, y):
            return False
    return True
