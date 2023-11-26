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
        np.array([0.611, 0.215, -0.826]),
        np.array([1.04771, -0.648117, -0.327335]),
        np.array([1.79603, -1.15619, -0.93251]),
        np.array([2.58726, -1.45961, -0.340022]),
        np.array([1.77129, -1.4912, -1.05943]),
        np.array([0.71349, -1.43365, -0.810231]),
        np.array([0, -1.5708, 0]),
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
