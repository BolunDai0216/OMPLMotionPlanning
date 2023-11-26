import pybullet as p
import pybullet_data


class Env:
    def __init__(self):
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

        # Load plane
        p.loadURDF("plane.urdf")

        # Load Robot
        robot_URDF = "robots/robot.urdf"
        self.robotID = p.loadURDF(robot_URDF, useFixedBase=True)

        # Load box
        box_URDF = "robots/box.urdf"
        box1 = p.loadURDF(box_URDF, useFixedBase=True)
        box2 = p.loadURDF(box_URDF, useFixedBase=True)
        box3 = p.loadURDF(box_URDF, useFixedBase=True)
        p.resetBasePositionAndOrientation(box1, [1.55, 1.0, 0.05], [0, 0, 0, 1])
        p.resetBasePositionAndOrientation(box2, [1.35, 0.2, 0.05], [0, 0, 0, 1])
        p.resetBasePositionAndOrientation(box3, [1.75, 0.2, 0.05], [0, 0, 0, 1])

    def reset(self):
        for joint_id, _joint_angle in enumerate([0.611, 0.215, -0.826]):
            p.resetJointState(self.robotID, joint_id, _joint_angle)
        p.stepSimulation()

    def step(self, q):
        for joint_id, _joint_angle in enumerate(q):
            p.resetJointState(self.robotID, joint_id, _joint_angle)
        p.stepSimulation()

    def close(self):
        p.disconnect()
