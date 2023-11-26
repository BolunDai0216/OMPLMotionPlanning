import hppfcl
import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper


class Robot:
    def __init__(self):
        robot_URDF = "robot.urdf"
        self.robot = RobotWrapper.BuildFromURDF(robot_URDF)

        self.link1_id = self.robot.model.getFrameId("link1")
        self.link2_id = self.robot.model.getFrameId("link2")
        self.link3_id = self.robot.model.getFrameId("link3")
        self.box_id = self.robot.model.getFrameId("box")

        self.T_offset = np.array(
            [[1, 0, 0, 0.25], [0, 1, 0, 0], [0, 0, 1, 0.05], [0, 0, 0, 1]]
        )
        self.T_box_offset = np.array(
            [[1, 0, 0, 0.2], [0, 1, 0, 0], [0, 0, 1, 0.05], [0, 0, 0, 1]]
        )

        self.link1_col = hppfcl.Box(np.array([0.5, 0.1, 0.1]))
        self.link2_col = hppfcl.Box(np.array([0.5, 0.1, 0.1]))
        self.link3_col = hppfcl.Box(np.array([0.5, 0.1, 0.1]))
        self.box_col = hppfcl.Box(np.array([0.4, 0.3, 0.1]))

    def update_pinocchio(self, q):
        dq = np.zeros(3)
        self.robot.computeJointJacobians(q)
        self.robot.framesForwardKinematics(q)
        self.robot.centroidalMomentum(q, dq)

    def get_state(self, q):
        self.update_pinocchio(q)

        T_joint_1 = self.robot.data.oMf[self.link1_id].homogeneous
        T_link_1 = pin.SE3(T_joint_1 @ self.T_offset)

        T_joint_2 = self.robot.data.oMf[self.link2_id].homogeneous
        T_link_2 = pin.SE3(T_joint_2 @ self.T_offset)

        T_joint_3 = self.robot.data.oMf[self.link3_id].homogeneous
        T_link_3 = pin.SE3(T_joint_3 @ self.T_offset)

        T_joint_box = self.robot.data.oMf[self.box_id].homogeneous
        T_link_box = pin.SE3(T_joint_box @ self.T_box_offset)

        info = {
            "link1_pos": T_link_1.translation,
            "link1_rot": T_link_1.rotation,
            "link2_pos": T_link_2.translation,
            "link2_rot": T_link_2.rotation,
            "link3_pos": T_link_3.translation,
            "link3_rot": T_link_3.rotation,
            "box_pos": T_link_box.translation,
            "box_rot": T_link_box.rotation,
        }

        return info

    def check_collision(self, info, col_list):
        T_link1 = hppfcl.Transform3f(info["link1_rot"], info["link1_pos"])
        T_link2 = hppfcl.Transform3f(info["link2_rot"], info["link2_pos"])
        T_link3 = hppfcl.Transform3f(info["link3_rot"], info["link3_pos"])
        T_box = hppfcl.Transform3f(info["box_rot"], info["box_pos"])

        link_cols = [
            (self.link1_col, T_link1),
            (self.link2_col, T_link2),
            (self.link3_col, T_link3),
            (self.box_col, T_box),
        ]

        cols = []
        for col in link_cols:
            for other_col in col_list:
                req = hppfcl.CollisionRequest()
                res = hppfcl.CollisionResult()
                cols.append(
                    hppfcl.collide(col[0], col[1], other_col[0], other_col[1], req, res)
                )

        return np.any(cols)
