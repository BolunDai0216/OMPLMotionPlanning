import pickle
import time
import pybullet

import numpy as np
from env import Env


def distance(v1, v2):
    return np.linalg.norm(np.array(v1) - np.array(v2))


def main():
    # with open("../data/path.pkl", "rb") as f:
    #     path = pickle.load(f)

    path = [
        [0.611, 0.215, -0.826],
        [1.35408, -0.384872, -1.32297],
        [1.96655, -0.0837118, -1.18028],
        [2.59736, 0.331914, -0.954574],
        [3.00716, 0.553205, -0.765477],
    ]

    env = Env()
    env.reset()
    target = pybullet.loadURDF("robots/target.urdf", useFixedBase=True)
    pybullet.resetBasePositionAndOrientation(target, [-1.5, 0.0, 0.05], [0, 0, 0, 1])

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
