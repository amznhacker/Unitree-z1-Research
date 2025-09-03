WARNING: Package name "aliengoZ1_description" does not follow the naming conventions. It should start with a lower case letter and only contain lower case letters, digits, underscores, and dashes.
[INFO] [1756932526.201628, 0.000000]: Controller Spawner: Waiting for service controller_manager/load_controller
[INFO] [1756932526.204042, 547.791800]: Controller Spawner: Waiting for service controller_manager/switch_controller
[INFO] [1756932526.205802, 547.793400]: Controller Spawner: Waiting for service controller_manager/unload_controller
[INFO] [1756932526.207574, 547.795200]: Loading controller: joint_state_controller
[INFO] [1756932526.211936, 547.799600]: Loading controller: Joint01_controller
[INFO] [1756932526.213555, 0.000000]: Loading model XML from ros parameter robot_description
[INFO] [1756932526.216930, 547.803800]: Waiting for service /gazebo/spawn_urdf_model
[INFO] [1756932526.218962, 547.806000]: Calling service /gazebo/spawn_urdf_model
[INFO] [1756932526.222136, 547.809800]: Spawn status: SpawnModel: Failure - entity already exists.
[ERROR] [1756932526.222762, 547.810400]: Spawn service failed. Exiting.
[gazebo-1] process has died [pid 36879, exit code 255, cmd /opt/ros/noetic/lib/gazebo_ros/gzserver -u -e ode /home/zero/catkin_ws/src/unitree_ros/unitree_gazebo/worlds/earth.world __name:=gazebo __log:=/home/zero/.ros/log/19b8981e-8906-11f0-a454-d94781f12508/gazebo-1.log].
log file: /home/zero/.ros/log/19b8981e-8906-11f0-a454-d94781f12508/gazebo-1*.log
[INFO] [1756932526.402855100]: Finished loading Gazebo ROS API Plugin.
[INFO] [1756932526.403546065]: waitForService: Service [/gazebo_gui/set_physics_properties] has not been advertised, waiting...
[urdf_spawner-3] process has died [pid 36889, exit code 1, cmd /opt/ros/noetic/lib/gazebo_ros/spawn_model -urdf -z 0.0 -model z1_gazebo -param robot_description -unpause __name:=urdf_spawner __log:=/home/zero/.ros/log/19b8981e-8906-11f0-a454-d94781f12508/urdf_spawner-3.log].
log file: /home/zero/.ros/log/19b8981e-8906-11f0-a454-d94781f12508/urdf_spawner-3*.log
[ERROR] [1756932527.216043, 548.795400]: Failed to load Joint01_controller

roslaunch unitree_gazebo z1.launch

