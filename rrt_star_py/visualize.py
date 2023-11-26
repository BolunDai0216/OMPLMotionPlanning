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
        [1.15153, -0.701572, -0.597796],
        [1.31188, -0.591199, -1.02931],
        [1.89611, -0.75533, -1.40862],
        [1.57089, -1.46605, -1.17068],
        [0.571719, -1.18631, -0.842414],
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
