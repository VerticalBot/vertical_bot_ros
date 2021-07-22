# vertical_bot_ros
start urdf model publisher:
ros2 launch vertical_robot_model model.launch.py

start node for set robot position and orientation
ros2 run vertical_robot_base_pkg position_control.py

topic for set robot position (type geometry_msgs/Pose ):
/robot_pose

for set joint position:
topic: /joint_states
type: sensor_msgs/msg/JointState
joints names: [joint_1, joint_2, joint_1_1, joint_1_2, joint_2_1, joint_2_2]
joint_1_2 = joint_1_1
joint_2_1 = joint_2_2
