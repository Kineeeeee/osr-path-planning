"""Random sampling of configurations."""
import numpy as np

def sample_free(env, n):
    """n random collision-free points, as an (n, 2) array."""
    points = []
    while len(points) < n:
        x = np.random.rand() * env.size_x
        y = np.random.rand() * env.size_y
        if not env.check_collision(x, y):
            points.append((x, y))
    return np.array(points)

def random_config(env):
    """A random point, may be in collision."""
    x = np.random.rand() * env.size_x
    y = np.random.rand() * env.size_y
    return np.array([x, y])
