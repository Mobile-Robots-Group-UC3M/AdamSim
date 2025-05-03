from pybullet_utils import bullet_client as bc
from pybullet_utils import urdfEditor as urdf_ed
import pybullet
import pybullet_data
import time
import os



# Create Bullet clients for three robots
p_robot = bc.BulletClient(connection_mode=pybullet.DIRECT)
p_robot.setAdditionalSearchPath(pybullet_data.getDataPath())

p_hand_r = bc.BulletClient(connection_mode=pybullet.DIRECT)
p_hand_r.setAdditionalSearchPath(pybullet_data.getDataPath())

p_hand_l = bc.BulletClient(connection_mode=pybullet.DIRECT)
p_hand_l.setAdditionalSearchPath(pybullet_data.getDataPath())

robot_urdf_path = "/home/vrosi/TFM/Adam_sim/paquetes_simulacion/rb1_base_description/robots/robot_wh.urdf"
r_hand_urdf_path = "/home/vrosi/TFM/Adam_sim/paquetes_simulacion/inspire_hands/inspire_hand_r/urdf/inspire_hand_r.urdf"
l_hand_urdf_path = "/home/vrosi/TFM/Adam_sim/paquetes_simulacion/inspire_hands/inspire_hand_l/urdf/inspire_hand_l.urdf"

r_hand = p_hand_r.loadURDF(r_hand_urdf_path)
l_hand = p_hand_l.loadURDF(r_hand_urdf_path)
robot = p_robot.loadURDF(robot_urdf_path)

# Create URDF editors
ed_robot = urdf_ed.UrdfEditor()
ed_robot.initializeFromBulletBody(robot, p_robot._client)

ed_hand_r = urdf_ed.UrdfEditor()
ed_hand_r.initializeFromBulletBody(r_hand, p_hand_r._client)

ed_hand_l = urdf_ed.UrdfEditor()
ed_hand_l.initializeFromBulletBody(l_hand, p_hand_l._client)

# Configuration for combining URDFs
parentLinkIndex = 26
jointPivotXYZInParent = [0, 0, 0]  # Adjust as needed
jointPivotRPYInParent = [0, 0, 0]

jointPivotXYZInChild = [0, 0, 0]
jointPivotRPYInChild = [0, 0, 0]

# Join the URDFs right_hand robot
try:
    new_joint_r = ed_robot.joinUrdf(
        ed_hand_r,
        parentLinkIndex,
        jointPivotXYZInParent,
        jointPivotRPYInParent,
        jointPivotXYZInChild,
        jointPivotRPYInChild,
        p_robot._client,
        p_hand_r._client,
    )
    new_joint_r.joint_type = p_robot.JOINT_FIXED
except Exception as e:
    raise RuntimeError(f"Failed to combine URDFs: {e}")



# Save the combined URDF
adam_rh_urdf_path = "adam_rh.urdf"
ed_robot.saveUrdf(adam_rh_urdf_path)


# Configuration for combining URDFs
parentLinkIndex = 50
jointPivotXYZInParent = [0, 0, 0]  # Adjust as needed
jointPivotRPYInParent = [0, 0, 0]

jointPivotXYZInChild = [0, 0, 0]
jointPivotRPYInChild = [0, 0, 0]

# Join the URDFs left_hand robot
try:
    new_joint_l = ed_robot.joinUrdf(
        ed_hand_l,
        parentLinkIndex,
        jointPivotXYZInParent,
        jointPivotRPYInParent,
        jointPivotXYZInChild,
        jointPivotRPYInChild,
        p_robot._client,
        p_hand_l._client,
    )
    new_joint_l.joint_type = p_robot.JOINT_FIXED
except Exception as e:
    raise RuntimeError(f"Failed to combine URDFs: {e}")

# Save the combined URDF
adam_urdf_path = "adam.urdf"
ed_robot.saveUrdf(adam_urdf_path)
