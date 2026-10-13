"""Exercise 1: solve the 2D motion planning problem with PRM."""
import argparse
import time

import pylab as pl

from planning import config
from planning.path import path_length
from planning.plotting import plot_prm
from planning.prm import prm
from planning.problem import make_problem


def main():
    parser = argparse.ArgumentParser(description="Exercise 1: PRM")
    parser.add_argument("--seed", type=int, default=4, help="environment and query (default 4)")
    parser.add_argument("--no-show", action="store_true", help="only save the figure")
    args = parser.parse_args()

    problem = make_problem(args.seed)
    if problem is None:
        raise SystemExit("Could not generate a query, try another seed")
    env, start, goal = problem

    t = time.perf_counter()
    path, nodes, adj = prm(env, start, goal, config.N_SAMPLES, config.RADIUS)
    elapsed = time.perf_counter() - t

    n_edges = sum(len(a) for a in adj) // 2
    print(f"PRM, seed {args.seed}: {len(nodes)} nodes, {n_edges} edges, {elapsed:.2f}s")
    if path is None:
        print("No path found")
    else:
        print(f"Path: {len(path)} points, length {path_length(path):.2f}")

    plot_prm(env, nodes, adj, path, start, goal)
    out = f"results/prm_seed{args.seed}.png"
    pl.savefig(out, dpi=150)
    print(f"Figure saved to {out}")
    if not args.no_show:
        pl.show(block=True)


if __name__ == "__main__":
    main()
