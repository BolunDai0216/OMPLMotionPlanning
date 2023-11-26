import numpy as np
from graph import Graph
from rich.progress import track
from rrt_utils import Scene, steer
from robot import Robot
import pinocchio as pin
import hppfcl


class RRTScene(Scene):
    def __init__(self, dimension, low, high, target, distance_threshold=0.1):
        super().__init__(
            dimension, low, high, target, distance_threshold=distance_threshold
        )
        self.robot = Robot()

        box1_quat = pin.Quaternion(1, 0, 0, 0)
        box2_quat = pin.Quaternion(1, 0, 0, 0)
        box3_quat = pin.Quaternion(1, 0, 0, 0)

        box1_R = box1_quat.toRotationMatrix()
        box2_R = box2_quat.toRotationMatrix()
        box3_R = box3_quat.toRotationMatrix()

        T_box1 = hppfcl.Transform3f(box1_R, np.array([1.45, 1.0, 0.05]))
        T_box2 = hppfcl.Transform3f(box2_R, np.array([1.25, 0.2, 0.05]))
        T_box3 = hppfcl.Transform3f(box3_R, np.array([1.65, 0.2, 0.05]))

        box1_col = hppfcl.Box(np.array([0.4, 0.3, 0.1]))
        box2_col = hppfcl.Box(np.array([0.4, 0.3, 0.1]))
        box3_col = hppfcl.Box(np.array([0.4, 0.3, 0.1]))

        self.box_cols = [(box1_col, T_box1), (box2_col, T_box2), (box3_col, T_box3)]

    def check_collision(self, v):
        """
        Check if a point is in collision with an obstacle
        """
        info = self.robot.get_state(v)
        in_collision = self.robot.check_collision(info, self.box_cols)

        return in_collision


def main():
    η = 10
    γ_rrt = 7
    graph = Graph(3, η=η, γ_rrt=γ_rrt)
    graph.set_root((0.611, 0.215, -0.826))

    scene = RRTScene(
        3,
        np.array([0, -np.pi / 2, -np.pi / 2]),
        np.array([np.pi, np.pi / 2, np.pi / 2]),
        (0.0, -np.pi / 2, 0.0),
    )

    for i in track(range(100000)):
        if np.random.rand() <= 0.1:
            v_rand = scene.target
        else:
            v_rand = scene.sample_free()

        v_nearest = graph.nearest(v_rand)[0]
        v_new = steer(v_nearest, v_rand, η)

        if scene.obstacle_free(v_nearest, v_new):
            V_near = graph.nearest(v_new, n=int(1e6), optimal=True)
            graph.add_vertex(v_new)
            v_min = v_nearest
            c_min = graph.cost(v_nearest) + graph.line_cost(v_nearest, v_new)

            # connect along a minimum-cost path
            for _v_near in V_near:
                collision_free = scene.obstacle_free(_v_near, v_new)
                c_new = graph.cost(_v_near) + graph.line_cost(_v_near, v_new)

                if collision_free and c_new < c_min:
                    v_min = _v_near
                    c_min = c_new

            graph.add_edge(v_min, v_new)

            # rewire the tree
            for _v_near in V_near:
                collision_free = scene.obstacle_free(_v_near, v_new)
                _c_near = graph.cost(v_new) + graph.line_cost(v_new, _v_near)
                c_near = graph.cost(_v_near)

                if collision_free and _c_near < c_near:
                    graph.change_parent(v_new, _v_near)

            succeed = scene.success(v_new)

            if succeed:
                print(f"Succeeded in {i} iterations!")
                path = graph.get_path(v_new)
                break

    breakpoint()


if __name__ == "__main__":
    main()
