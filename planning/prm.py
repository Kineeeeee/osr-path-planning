"""PRM (exercise 1): build a roadmap once, then answer queries on it."""
import numpy as np

from .collision import is_segment_free
from .graph_search import dijkstra
from .sampling import sample_free

def build_roadmap(env, n_samples, radius):
    """Sample nodes and connect every pair within the radius whose segment is free.

    Returns (nodes, adj): nodes is an (N, 2) array, adj[i] a list of (j, distance).
    """
    nodes = sample_free(env, n_samples)
    adj = [[] for _ in range(len(nodes))]
    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            d = np.linalg.norm(nodes[i] - nodes[j])
            if d <= radius and is_segment_free(env, nodes[i], nodes[j]):  # cheap test first
                adj[i].append((j, d))
                adj[j].append((i, d))
    return nodes, adj


def query(env, nodes, adj, start, goal, radius):
    """Connect start and goal to the roadmap, then search it with Dijkstra.

    Returns [start, ..., goal] or None. Works on a copy, so the roadmap can be reused.
    """
    if is_segment_free(env, start, goal):  # a straight line is the shortest path
        return [tuple(start), tuple(goal)]

    # start is node N, goal is node N + 1
    N = len(nodes)
    all_nodes = np.vstack([nodes, start, goal])
    all_adj = [list(a) for a in adj] + [[], []]
    for i in (N, N + 1):
        for j in range(i):  # for goal, this includes start
            d = np.linalg.norm(all_nodes[i] - all_nodes[j])
            if d <= radius and is_segment_free(env, all_nodes[i], all_nodes[j]):
                all_adj[i].append((j, d))
                all_adj[j].append((i, d))

    indices = dijkstra(all_adj, N, N + 1)
    if indices is None:
        return None
    return [tuple(all_nodes[i]) for i in indices]


def prm(env, start, goal, n_samples, radius):
    """Single query: build a roadmap and search it. Returns (path, nodes, adj)."""
    nodes, adj = build_roadmap(env, n_samples, radius)
    path = query(env, nodes, adj, start, goal, radius)
    return path, nodes, adj
