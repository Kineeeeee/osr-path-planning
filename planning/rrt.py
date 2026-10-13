"""RRT (exercise 2): grow a tree from start until it reaches goal."""
import numpy as np

from .collision import is_segment_free
from .sampling import random_config


def nearest(nodes, q):
    """Index of the tree node closest to q."""
    dists = np.linalg.norm(np.array(nodes) - q, axis=1)
    return int(np.argmin(dists))


def steer(env, q_near, q_rand, step_size):
    """Move from q_near towards q_rand by at most step_size."""
    d = np.linalg.norm(q_near - q_rand)
    if d <= step_size:
        q_new = q_rand
    else:
        q_new = q_near + (q_rand - q_near) / d * step_size
    if is_segment_free(env, q_near, q_new):
        return q_new
    return None


def build_rrt(env, start, goal, step_size, max_iter):
    """Grow the tree until a new node connects to goal."""
    start, goal = np.array(start, dtype=float), np.array(goal, dtype=float)
    nodes, parent = [start], [None]

    if is_segment_free(env, start, goal):  # same straight-line check as PRM
        nodes.append(goal)
        parent.append(0)
        return nodes, parent, True

    for _ in range(max_iter):
        q_rand = random_config(env)
        i_near = nearest(nodes, q_rand)
        q_new = steer(env, nodes[i_near], q_rand, step_size)
        if q_new is None:
            continue
        nodes.append(q_new)
        parent.append(i_near)
        if np.linalg.norm(q_new - goal) <= step_size and is_segment_free(env, q_new, goal):
            parent.append(len(nodes) - 1)  # index of q_new, taken before appending goal
            nodes.append(goal)
            return nodes, parent, True
    return nodes, parent, False


def extract_path(nodes, parent):
    """Follow parents from goal (last node) back to start. Returns [start, ..., goal]."""
    u = len(nodes) - 1
    path = []
    while u is not None:
        path.append(u)
        u = parent[u]
    path.reverse()
    return [tuple(nodes[i]) for i in path]


def rrt(env, start, goal, step_size, max_iter):
    """Single query. Returns (path, nodes, parent); path is None if not found."""
    nodes, parent, found = build_rrt(env, start, goal, step_size, max_iter)
    path = extract_path(nodes, parent) if found else None
    return path, nodes, parent
