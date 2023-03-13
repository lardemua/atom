Launch calibration studio:

    roslaunch atom_worlds calibration_studio.launch

Spawn your robot.

Other options:
* launch_rviz - Whether to launch RViz or not.
* rviz_config - RViz configuration file.
* gui - Starts gazebo gui.
* world_name - World on Gazebo.
* paused - Starts gazebo in paused mode.

_________________

Run **interactive** pattern:

    rosrun atom_worlds interactive_pattern

Run **autonomous** pattern:

    rosrun atom_worlds autonomous_pattern

    rosrun atom_worlds autonomous_pattern -vi

Configure the camera topic on **camera.yaml**

_________________

Launch rosbag record, after configure the topics to record:

    roslaunch atom_worlds record_sensor_data.launch