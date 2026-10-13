"""Exercise 3: post-process the PRM and RRT paths with path shortcutting."""
import argparse

import numpy as np
import pylab as pl

from planning import config
from planning.path import path_length
from planning.plotting import plot_shortcut
from planning.prm import prm
from planning.problem import make_problem
from planning.rrt import rrt
from planning.shortcut import shortcut


def main():
    parser = argparse.ArgumentParser(description="Exercise 3: path shortcutting")
    parser.add_argument("--seed", type=int, default=4, help="environment and query (default 4)")
    parser.add_argument("--no-show", action="store_true", help="only save the figures")
    args = parser.parse_args()

    problem = make_problem(args.seed)
    if problem is None:
        raise SystemExit("Could not generate a query, try another seed")

    # rebuild the problem before each planner
    env, start, goal = problem
    path_prm = prm(env, start, goal, config.N_SAMPLES, config.RADIUS)[0]
    env, start, goal = make_problem(args.seed)
    path_rrt = rrt(env, start, goal, config.STEP_SIZE, config.MAX_ITER)[0]

    np.random.seed(0)  # separate seed for shortcutting
    for name, path in (("prm", path_prm), ("rrt", path_rrt)):
        if path is None:
            print(f"{name.upper()}: no path to shortcut")
            continue
        new_path = shortcut(env, path, config.MAXREP)
        print(f"{name.upper()}: length {path_length(path):.2f} -> {path_length(new_path):.2f}, "
              f"{len(path)} -> {len(new_path)} points")

        pl.figure()
        plot_shortcut(env, path, new_path, start, goal,
                      f"{name.upper()} path, shortcutting with maxrep = {config.MAXREP}")
        out = f"results/shortcut_{name}_seed{args.seed}.png"
        pl.savefig(out, dpi=150)
        print(f"Figure saved to {out}")

    if not args.no_show:
        pl.show(block=True)


if __name__ == "__main__":
    main()
