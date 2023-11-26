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
        [1.26546, -0.654499, -0.824362],
        [2.13205, -1.30689, -0.736241],
        [2.11107, -0.833041, -1.0208],
        [1.54722, -1.55515, -1.29628],
        [1.4129, -1.53362, -0.732023],
        [0.945826, -1.56342, -1.14806],
        [0.874535, -0.988174, -1.21177],
        [0.679781, -1.27826, -0.181108],
        [0.289949, -1.35163, 0.32477],
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
