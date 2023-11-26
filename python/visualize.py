import pickle
import time

import numpy as np
from env import Env


def distance(v1, v2):
    return np.linalg.norm(np.array(v1) - np.array(v2))


def main():
    # with open("../data/path.pkl", "rb") as f:
    #     path = pickle.load(f)

    path = [
        [0.611, 0.215, -0.826],
        [1.26761, -0.311073, -1.28067],
        [1.67032, -0.751374, -1.49139],
        [1.06861, -1.08121, -1.53841],
        [0.141754, -1.12796, -1.22671],
        [0, -1.5708, 0],
    ]

    env = Env()
    env.reset()
    n = len(path)
    distance_threshold = 0.01

    for i in range(n - 1):
        v1 = path[i]
        v2 = path[i + 1]

        num_points = int(np.ceil(distance(v1, v2) / distance_threshold)) + 1

        values = []

        for j in range(3):
            values.append(np.linspace(v1[j], v2[j], num_points))

        # stack arrays together for nD points
        points = np.stack(values, axis=-1)  # (num_points, nD)

        for p in points[:-1, :]:
            env.step(p)
            time.sleep(0.02)

    for i in range(1000):
        env.step(points[-1, :])
        time.sleep(0.01)

    env.close()


if __name__ == "__main__":
    main()
