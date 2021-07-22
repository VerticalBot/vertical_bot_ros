# vertical_bot_ros
start urdf model publisher:\n
ros2 launch vertical_robot_model model.launch.py\n

start node for set robot position and orientation\n
ros2 run vertical_robot_base_pkg position_control.py\n

topic for set robot position (type geometry_msgs/Pose ):\n
/robot_pose\n

for set joint position:\n
topic: /joint_states\n
type: sensor_msgs/msg/JointState\n
joints names: [joint_1, joint_2, joint_1_1, joint_1_2, joint_2_1, joint_2_2]\n
joint_1_2 = joint_1_1\n
joint_2_1 = joint_2_2\n
