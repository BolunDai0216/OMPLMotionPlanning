import numpy as np
from rtree import index


class Graph:
    """
    A class to represent a graph. The graph is represented by a vertex set V
    and an edge set E. The graph is stored in an RTree Index for fast nearest
    neighbor queries. The graph is initialized with a dimension, which is the
    dimension of the space the graph is embedded in. The graph is initialized
    with an empty vertex set and an empty edge set. The graph is initialized
    with a size of 0.
    """

    def __init__(self, dimension, η=2, γ_rrt=7, name="graph"):
        """
        Initialize the graph.

        Args:
            dimension (int): The dimension of the graph.
            η (float): The step length.
            γ_rrt (float): scaling factor for the RRT* distance check.
        """
        self.root = None
        self.dimension = dimension
        self.η = η
        self.γ_rrt = γ_rrt
        self.name = name

        prop = index.Property()
        prop.dimension = dimension
        self.V = index.Index(interleaved=True, properties=prop)
        self.size = 0
        self.E = dict()

    def set_root(self, v_root):
        """
        Set the root of the graph.

        Args:
            v (tuple): The vertex to be set as the root of the graph.
        """
        self.root = v_root
        self.add_vertex(v_root)

    def add_vertex(self, v):
        """
        Add a vertex to the graph.

        Args:
            v (tuple): The vertex to be added to the graph.
        """
        self.V.insert(0, v + v, obj=v)
        self.size += 1

    def add_edge(self, v_parent, v_child):
        """
        Add an edge to the graph.

        Args:
            v_parent (tuple): The parent vertex.
            v_child (tuple): The child vertex.
        """
        self.E[v_child] = v_parent

    def remove_vertex(self, v):
        """
        Remove a vertex from the graph.

        Args:
            v (tuple): The vertex to be removed from the graph.
        """
        self.V.delete(0, v + v)
        self.size -= 1

    def remove_edge(self, v_child):
        """
        Remove an edge from the graph.

        Args:
            v_child (tuple): The child vertex.
        """
        del self.E[v_child]

    def nearest(self, v, n=1, optimal=False):
        """
        Return the n nearest vertices to v.

        Args:
            v (tuple): The vertex to find the nearest vertices to.
            n (int): The number of nearest vertices to return.
            optimal (bool): Whether to use the RRT* distance check or not.
        """
        _near = self.V.nearest(v + v, n, objects=True)
        near = []
        for _n in _near:
            if optimal:
                _threshold = self.γ_rrt * (
                    (np.log(self.size + 1) / (self.size + 1)) ** (1 / self.dimension)
                )
                threshold = np.min([_threshold, self.η]) + 1e-3
                if self.distance(_n.object, v) <= threshold:
                    near.append(_n.object)
            else:
                near.append(_n.object)

        return near

    def change_parent(self, v_new_parent, v_child):
        """
        Change the parent of a vertex.

        Args:
            v_new_parent (tuple): The new parent vertex.
            v_child (tuple): The child vertex.
        """
        self.E[v_child] = v_new_parent

    def distance(self, v1, v2):
        """
        Return the distance between two vertices.

        Args:
            v1 (tuple): The first vertex.
            v2 (tuple): The second vertex.
        """
        return np.linalg.norm(np.array(v1) - np.array(v2))

    def cost(self, v):
        """
        Return the cost of a vertex.

        Args:
            v (tuple): The vertex to find the cost of.
        """
        cost = 0

        while v != self.root:
            v_parent = self.E[v]
            cost += self.distance(v, v_parent)
            v = v_parent

        return cost

    def line_cost(self, v1, v2):
        """
        Return the cost of a line between two vertices.

        Args:
            v1 (tuple): The first vertex.
            v2 (tuple): The second vertex.
        """
        return self.distance(v1, v2)

    def get_path(self, v):
        """
        Return the path from a vertex to the root.

        Args:
            v (tuple): The vertex to find the path from.

        Returns:
            list: The path from the vertex to the root.
        """
        path = [v]

        while v != self.root:
            v_parent = self.E[v]
            path.append(v_parent)
            v = v_parent

        return path[::-1]
