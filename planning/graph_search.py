"""Shortest path on a weighted graph."""
import heapq
import numpy as np

def dijkstra(adj, source, target):
    """Shortest path from source to target; adj[u] is a list of (v, weight).

    Returns the node indices [source, ..., target], or None if target is unreachable.
    """
    dist = {source: 0.0}
    parent = {source: None}
    heap = [(0.0, source)]
    while heap:
        d, u = heapq.heappop(heap)
        if u == target:
            break
        if d > dist[u]:  # outdated heap entry
            continue
        for v, w in adj[u]:
            if d + w < dist.get(v, np.inf):
                dist[v] = d + w
                parent[v] = u
                heapq.heappush(heap, (d + w, v))
    if target not in parent:
        return None

    # reconstruct path
    path = []
    u = target
    while u is not None:
        path.append(u)
        u = parent[u]
    path.reverse()
    return path
