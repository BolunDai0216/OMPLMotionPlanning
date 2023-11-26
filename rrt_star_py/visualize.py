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
        [1.38249, -0.44094, -1.22463],
        [1.91728, -0.00495815, -0.38304],
        [1.83292, -0.745326, 0.410113],
        [2.36045, -0.788, 0.212966],
        [2.57365, -0.998871, 0.0373279],
        [2.62035, -1.25491, -0.585489],
        [2.07962, -1.30679, -1.04546],
        [1.75713, -1.33056, -1.45125],
        [1.40113, -1.41991, -0.921711],
        [1.19986, -1.47089, -1.09136],
        [0.692857, -1.4082, -1.04819],
        [0.756871, -1.18433, -0.907123],
        [0.577455, -1.25829, -0.518105],
        [0.770384, -1.43784, -0.424845],
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
