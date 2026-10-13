"""Exercise 2 (comparison): running time of PRM and RRT, single query and multiple queries."""
import time

import numpy as np

import environment_2d
from planning import config
from planning.collision import is_segment_free
from planning.prm import build_roadmap, prm, query
from planning.problem import make_problem
from planning.rrt import rrt


def single_query(seeds):
    """One new environment and one query per seed. PRM builds a new roadmap each time."""
    rows = []
    for seed in seeds:
        problem = make_problem(seed)
        if problem is None:
            continue
        env, start, goal = problem
        direct = is_segment_free(env, start, goal)

        t0 = time.perf_counter()
        path_prm = prm(env, start, goal, config.N_SAMPLES, config.RADIUS)[0]
        prm_time = time.perf_counter() - t0

        # Rebuild the problem so RRT gets the same random numbers as exercise2_rrt.py
        env, start, goal = make_problem(seed)
        t0 = time.perf_counter()
        path_rrt = rrt(env, start, goal, config.STEP_SIZE, config.MAX_ITER)[0]
        rrt_time = time.perf_counter() - t0

        rows.append({"seed": seed, "direct": direct,
                     "prm_time": prm_time, "prm_found": path_prm is not None,
                     "rrt_time": rrt_time, "rrt_found": path_rrt is not None})
    return rows


def multi_query(seed, n_queries):
    """One environment, n_queries queries. PRM builds the roadmap once; RRT starts over each time."""
    np.random.seed(seed)
    env = environment_2d.Environment(10, 6, 5)
    queries = []
    while len(queries) < n_queries:
        q = env.random_query()
        if q is not None:
            queries.append((q[:2], q[2:]))

    t0 = time.perf_counter()
    nodes, adj = build_roadmap(env, config.N_SAMPLES, config.RADIUS)
    prm_build = time.perf_counter() - t0

    prm_queries, prm_found = 0, 0
    for start, goal in queries:
        t0 = time.perf_counter()
        path = query(env, nodes, adj, start, goal, config.RADIUS)
        prm_queries += time.perf_counter() - t0
        prm_found += path is not None

    rrt_total, rrt_found = 0, 0
    for start, goal in queries:
        t0 = time.perf_counter()
        path = rrt(env, start, goal, config.STEP_SIZE, config.MAX_ITER)[0]
        rrt_total += time.perf_counter() - t0
        rrt_found += path is not None

    return {"n_queries": n_queries, "prm_build": prm_build, "prm_queries": prm_queries,
            "prm_found": prm_found, "rrt_total": rrt_total, "rrt_found": rrt_found}


def print_single(rows):
    print("=== Single query: each seed = new environment, one query ===")
    print(f"{'seed':>4} {'direct':>6} {'PRM (s)':>8} {'found':>5} {'RRT (s)':>8} {'found':>5}")
    for r in rows:
        print(f"{r['seed']:>4} {str(r['direct']):>6} "
              f"{r['prm_time']:>8.3f} {str(r['prm_found']):>5} "
              f"{r['rrt_time']:>8.3f} {str(r['rrt_found']):>5}")

    # Direct queries: RRT returns at once while PRM still builds a roadmap,
    # so the main comparison is the "blocked" group.
    for label, group in (("direct", [r for r in rows if r["direct"]]),
                         ("blocked", [r for r in rows if not r["direct"]])):
        if not group:
            continue
        n = len(group)
        print(f"{label:>7} ({n} queries): "
              f"PRM mean {np.mean([r['prm_time'] for r in group]):.3f}s, "
              f"found {sum(r['prm_found'] for r in group)}/{n} | "
              f"RRT mean {np.mean([r['rrt_time'] for r in group]):.3f}s, "
              f"found {sum(r['rrt_found'] for r in group)}/{n}")


def print_multi(res):
    n = res["n_queries"]
    prm_total = res["prm_build"] + res["prm_queries"]
    print(f"=== Multi query: one environment, {n} queries ===")
    print(f"PRM: build {res['prm_build']:.3f}s + queries {res['prm_queries']:.3f}s "
          f"= {prm_total:.3f}s, found {res['prm_found']}/{n}")
    print(f"RRT: total {res['rrt_total']:.3f}s, found {res['rrt_found']}/{n}")
    print(f"Per query: PRM {prm_total / n:.3f}s, RRT {res['rrt_total'] / n:.3f}s")


if __name__ == "__main__":
    print_single(single_query(range(10)))
    print()
    print_multi(multi_query(seed=4, n_queries=20))
