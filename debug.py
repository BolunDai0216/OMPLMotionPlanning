import hppfcl
import numpy as np
import pinocchio as pin
import pybullet as p
import pybullet_data

from robot import Robot


def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # Load plane
    p.loadURDF("plane.urdf")

    # Load Robot
    robot_URDF = "robot.urdf"
    robotID = p.loadURDF(robot_URDF, useFixedBase=True)

    # Load box
    box_URDF = "box.urdf"
    box1 = p.loadURDF(box_URDF, useFixedBase=True)
    box2 = p.loadURDF(box_URDF, useFixedBase=True)
    box3 = p.loadURDF(box_URDF, useFixedBase=True)
    p.resetBasePositionAndOrientation(box1, [1.5, 0.5, 0.05], [0, 0, 0, 1])
    p.resetBasePositionAndOrientation(box2, [1.0, 0.0, 0.05], [0, 0, 0, 1])
    p.resetBasePositionAndOrientation(box3, [1.5, 0.0, 0.05], [0, 0, 0, 1])

    box1_quat = pin.Quaternion(1, 0, 0, 0)
    box2_quat = pin.Quaternion(1, 0, 0, 0)
    box3_quat = pin.Quaternion(1, 0, 0, 0)

    box1_R = box1_quat.toRotationMatrix()
    box2_R = box2_quat.toRotationMatrix()
    box3_R = box3_quat.toRotationMatrix()

    T_box1 = hppfcl.Transform3f(box1_R, np.array([1.5, 0.5, 0.05]))
    T_box2 = hppfcl.Transform3f(box2_R, np.array([1.0, 0.0, 0.05]))
    T_box3 = hppfcl.Transform3f(box3_R, np.array([1.5, 0.0, 0.05]))

    box1_col = hppfcl.Box(np.array([0.4, 0.3, 0.1]))
    box2_col = hppfcl.Box(np.array([0.4, 0.3, 0.1]))
    box3_col = hppfcl.Box(np.array([0.4, 0.3, 0.1]))
    box_cols = [(box1_col, T_box1), (box2_col, T_box2), (box3_col, T_box3)]

    # Get number of joints
    n_j = p.getNumJoints(robotID)

    debug_sliders = []
    joint_ids = []

    default_joint_angles = [0.0, 0.0, 0.0]
    counter = 0

    robot = Robot()

    for i in range(n_j):
        # get info of each joint
        _joint_infos = p.getJointInfo(robotID, i)

        if _joint_infos[2] != p.JOINT_FIXED:
            # Add a debug slider for all non-fixed joints
            debug_sliders.append(
                p.addUserDebugParameter(
                    _joint_infos[1].decode("UTF-8"),  # Joint Name
                    _joint_infos[8],  # Lower Joint Limit
                    _joint_infos[9],  # Upper Joint Limit
                    default_joint_angles[counter],  # Default Joint Angle
                )
            )

            # Save the non-fixed joint IDs
            joint_ids.append(_joint_infos[0])
            counter += 1

    while True:
        q = np.zeros(3)

        for slider_id, joint_id in zip(debug_sliders, joint_ids):
            # Get joint angle from debug slider
            try:
                _joint_angle = p.readUserDebugParameter(slider_id)
            except:
                # Sometimes it fails to read the debug slider
                continue

            # Apply joint angle to robot
            p.resetJointState(robotID, joint_id, _joint_angle)
            q[joint_id] = _joint_angle

        info = robot.get_state(q)
        in_collsion = robot.check_collision(info, box_cols)

        if in_collsion:
            print("Collision detected!")
        else:
            print("No collision detected!")

        p.stepSimulation()

    p.disconnect()


if __name__ == "__main__":
    main()
