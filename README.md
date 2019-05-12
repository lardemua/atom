# AtlasCarCalibration
Reading the sensors starting position of a robot xacro file (atlas car sample) and create interactive markers associated to them.
Saving the markers final position and reopen the robot with the sensors at the updated position.

# Installation
First, you need to uninstall urdfdom_py ros package and install the updated version of this package so that can be able to recognize sensor element.
To remove the old version, run the command:
```
sudo apt-get remove ros-melodic-urdfdom_py
```
Then, you must clone the new version to your catkin source:
```
cd && cd catkin_ws/src
git clone https://github.com/afonsocastro/urdf_parser_py.git
```
Beside this, it will be needed some sensors packages.
In the same directory (catkin_ws/src):
```
git clone https://github.com/clearpathrobotics/LMS1xx.git
git clone https://github.com/ros-drivers/pointgrey_camera_driver.git
git clone https://github.com/SICKAG/sick_ldmrs_laser.git
```
Also, in the same directory (catkin_ws/src), clone the atlas car model:
````
git clone https://github.com/lardemua/atlas-core.git
```` 

# Usage Instructions
First , add this package to your catkin workspace source.

Run the command:
```
roslaunch interactive_marker_test rviz.launch read_first_guess:=false
```

Now rviz will open, and you will be able to see the atlas car model with the sensors in their first position.

Then, in a new terminal:
```
rosrun interactive_marker_test create_first_guess.py -f car_center
```

Now you can move the green markers and save the new sensors configuration.
Kill the booth process in the terminals, and run:

```
roslaunch interactive_marker_test rviz.launch read_first_guess:=true
```
Now you will see the atlas car model with the sensors in the updated position.

# Known problems

## urdf model not showing on rviz

If you can't see the car on rviz (run this command before everything):

```
export LC_NUMERIC="en_US.UTF-8"
```

If this worked out, you must run always this command at the first place. To avoid this constant work, you can always
add this command to your bashrc file (copy and paste at the end of the bashrc document).
Now you don't have to worry about this anymore!
