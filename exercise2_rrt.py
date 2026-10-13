"""Exercise 2: solve the 2D motion planning problem with RRT."""
import argparse
import time

import pylab as pl

from planning import config
from planning.path import path_length
from planning.plotting import plot_rrt
from planning.problem import make_problem
from planning.rrt import rrt


def main():
    parser = argparse.ArgumentParser(description="Exercise 2: RRT")
    parser.add_argument("--seed", type=int, default=4, help="environment and query (default 4)")
    parser.add_argument("--no-show", action="store_true", help="only save the figure")
    args = parser.parse_args()

    problem = make_problem(args.seed)
    if problem is None:
        raise SystemExit("Could not generate a query, try another seed")
    env, start, goal = problem

    t = time.perf_counter()
    path, nodes, parent = rrt(env, start, goal, config.STEP_SIZE, config.MAX_ITER)
    elapsed = time.perf_counter() - t

    print(f"RRT, seed {args.seed}: {len(nodes)} nodes, {elapsed:.2f}s")
    if path is None:
        print(f"No path found after {config.MAX_ITER} iterations")
    else:
        print(f"Path: {len(path)} points, length {path_length(path):.2f}")

    plot_rrt(env, nodes, parent, path, start, goal)
    out = f"results/rrt_seed{args.seed}.png"
    pl.savefig(out, dpi=150)
    print(f"Figure saved to {out}")
    if not args.no_show:
        pl.show(block=True)


if __name__ == "__main__":
    main()
