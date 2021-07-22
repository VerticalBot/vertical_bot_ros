# vertical_bot_ros
start urdf model publisher:
ros2 launch vertica_robot_model model.launch.py

start node for set robot position and orientation
ros2 run vertical_robot_base_pkg position_control.py

topic for set robot position (type geometry_msgs/Pose ):
/robot_pose
