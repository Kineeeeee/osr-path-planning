"""Figures for the three exercises."""
import numpy as np
import pylab as pl
from matplotlib.collections import LineCollection

from .path import path_length

def plot_prm(env, nodes, adj, path, start, goal):
    """Environment, roadmap (grey) and path (green)."""
    pl.clf()
    env.plot()
    # Each edge is stored twice in adj, keep i < j.
    edges = [(nodes[i], nodes[j]) for i, neighbors in enumerate(adj) for j, _ in neighbors if i < j]
    pl.gca().add_collection(LineCollection(edges, colors="0.8", linewidths=0.5))
    pl.plot(nodes[:, 0], nodes[:, 1], "k.", markersize=2)
    _plot_path_and_query(env, path, start, goal)
    pl.title(f"PRM: {len(nodes)} nodes, {len(edges)} edges, {_status(path)}")
    pl.axis("equal")


def plot_rrt(env, nodes, parent, path, start, goal):
    """Environment, tree (grey) and path (green)."""
    pl.clf()
    env.plot()
    edges = [(nodes[parent[i]], nodes[i]) for i in range(1, len(nodes))]
    pl.gca().add_collection(LineCollection(edges, colors="0.6", linewidths=0.6))
    nodes_arr = np.array(nodes)
    pl.plot(nodes_arr[:, 0], nodes_arr[:, 1], "k.", markersize=2)
    _plot_path_and_query(env, path, start, goal)
    pl.title(f"RRT: {len(nodes)} nodes, {_status(path)}")
    pl.axis("equal")

def plot_shortcut(env, before, after, start, goal, title):
    """Path before (dashed) and after (green) shortcutting."""
    pl.clf()
    env.plot()
    b, a = np.array(before), np.array(after)
    pl.plot(b[:, 0], b[:, 1], "--", color="0.5", linewidth=1.2,
            label=f"before: {len(before)} points, length {path_length(before):.2f}")
    pl.plot(a[:, 0], a[:, 1], "g-", linewidth=2,
            label=f"after: {len(after)} points, length {path_length(after):.2f}")
    env.plot_query(start[0], start[1], goal[0], goal[1])
    pl.title(title)
    pl.legend(loc="upper center", bbox_to_anchor=(0.5, -0.08), ncol=2, fontsize=8)
    pl.axis("equal")
    pl.tight_layout()

def _plot_path_and_query(env, path, start, goal):
    if path is not None:
        path = np.array(path)
        pl.plot(path[:, 0], path[:, 1], "g-", linewidth=2)
    env.plot_query(start[0], start[1], goal[0], goal[1])  # on top of everything

def _status(path):
    return f"{len(path)} waypoints" if path is not None else "no path found"
