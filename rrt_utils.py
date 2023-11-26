import numpy as np


def steer(v1, v2, η):
    """
    Go from v1 to v2 with a maximum distance of η

    Args:
        v1 (tuple): The starting vertex.
        v2 (tuple): The end vertex.
        η (float): The maximum distance to travel.
    """
    v1_arr = np.array(v1)
    v2_arr = np.array(v2)

    diff = v2_arr - v1_arr
    dist = np.linalg.norm(diff)
    diff_vec = diff / dist

    scale = np.clip(dist, 0, η)
    v_arr_new = v1_arr + scale * diff_vec
    v_new = tuple(v_arr_new)

    return v_new


class Scene:
    def __init__(self, dimension, low, high, target, distance_threshold=0.1):
        """
        Args:
            dimension (int): The dimension of the data.
            low (float): The lower bound of the data.
            high (float): The upper bound of the data.
        """
        self.dimension = dimension
        self.low = low
        self.high = high
        self.target = target
        self.distance_threshold = distance_threshold

    def sample_free(self):
        """
        Sample a point from the scene.

        Returns:
            tuple: The sampled point.
        """
        while True:
            v = np.random.uniform(low=self.low, high=self.high, size=(self.dimension,))
            if not self.check_collision(v):
                break

        return tuple(v)

    def obstacle_free(self, v1, v2):
        """
        Check if the line between two points is obstacle free.

        Args:
            v1 (tuple): The first point.
            v2 (tuple): The second point.

        Returns:
            bool: True if the line is obstacle free, False otherwise.
        """
        num_points = int(np.ceil(self.distance(v1, v2) / self.distance_threshold)) + 1

        values = []

        for i in range(self.dimension):
            values.append(np.linspace(v1[i], v2[i], num_points))

        # stack arrays together for nD points
        points = np.stack(values, axis=-1)  # (num_points, nD)

        for point in points:
            if self.check_collision(point):
                return False

        return True

    def check_collision(self, v):
        raise NotImplementedError

    def distance(self, v1, v2):
        """
        Euclidean distance between two vertices.

        Args:
            v1 (tuple): The first vertex.
            v2 (tuple): The second vertex.

        Returns:
            float: The distance between the two vertices.
        """
        v1 = np.array(v1)
        v2 = np.array(v2)

        return np.linalg.norm(v1 - v2)

    def success(self, v):
        """
        Check if a vertex is within the target region.

        Args:
            v (tuple): The vertex to check.

        Returns:
            bool: True if the vertex is within the target region, False otherwise.
        """
        succeed = self.distance(v, self.target) <= self.distance_threshold

        return succeed

    def cost_to_go(self, v):
        """
        Cost to go from a vertex to the target.

        Args:
            v (tuple): The vertex to check.

        Returns:
            float: The cost to go from the vertex to the target.
        """
        return self.distance(v, self.target)
