"""Default parameters, shared by all exercise scripts."""

# PRM (exercise 1).
N_SAMPLES = 1000   # roadmap nodes
RADIUS = 0.6       # connection radius

# RRT (exercise 2)
STEP_SIZE = 0.3    # max length of one extension (r in the exercise)
MAX_ITER = 10000   # iterations before giving up

# Path shortcutting (exercise 3)
MAXREP = 200       # shortcut attempts

# Collision check of a segment, as suggested by the exercise: call
# env.check_collision on points this far apart along the segment.
COLLISION_STEP = 0.001
