# vertical_bot_ros
start urdf model publisher:
ros2 launch vertical_robot_model model.launch.py

start node for set robot position and orientation
ros2 run vertical_robot_base_pkg position_control.py

topic for set robot position (type geometry_msgs/Pose ):
/robot_pose

for set joint position:\
topic: /joint_states\
type: sensor_msgs/msg/JointState\
joints names: [joint_1, joint_2, joint_1_1, joint_1_2, joint_2_1, joint_2_2]\
joint_1_2 = joint_1_1\
joint_2_1 = joint_2_2


Start platform with manipulator:
ros2 launch vertical_robot_model model.launch.py

    manipulator control:

        cartesian spase - topic: /cmd_point, type: std_msgs/msg/Float32MultiArray, data:[x, y, z]

        joint spase - topic: /cmd_joint_state, type: sensor_msgs/msg/JointState, example: msg.position = [0.0, 0.0, 0.0, 0.0]

        robot state - topic: /palletizer_robot_state, type: std_msgs/msg/Int32, info: 0 - robot does not move, 1 - robot is moving
