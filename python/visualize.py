import argparse
import pickle
import time

import numpy as np
import pybullet
from env import Env


def distance(v1, v2):
    return np.linalg.norm(np.array(v1) - np.array(v2))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--from-pickle", action="store_true")
    args = parser.parse_args()

    env = Env()
    env.reset()

    if args.from_pickle:
        with open("../data/path.pkl", "rb") as f:
            path = pickle.load(f)
    else:
        path = np.genfromtxt("../data/plan.csv", delimiter=",")

        target = pybullet.loadURDF("robots/target.urdf", useFixedBase=True)
        pybullet.resetBasePositionAndOrientation(
            target, [-1.5, 0.0, 0.05], [0, 0, 0, 1]
        )

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
