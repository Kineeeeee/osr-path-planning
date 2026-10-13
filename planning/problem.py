"""Environment and query, as in the exercise's sample code."""
import numpy as np

import environment_2d


def make_problem(seed, n_obs=5):
    """ Create a 10 x 6 map with n_obs random triangles and one random query."""
    np.random.seed(seed)
    env = environment_2d.Environment(10, 6, n_obs)
    q = env.random_query()
    if q is None:
        return None
    x_start, y_start, x_goal, y_goal = q
    return env, (x_start, y_start), (x_goal, y_goal)
