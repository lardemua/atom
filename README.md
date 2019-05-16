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

## Using PR2 robot instead of AtlasCar2
If you want to try it with the PR2 robot model too, it is needed to have the robot xacro files on your catkin source.
For that, clone the package:

```
git clone https://github.com/PR2/pr2_common.git
```

# Usage Instructions
First , add this package to your catkin workspace source.

Run the command:
```
roslaunch interactive_marker_test rviz.launch car_model:=true read_first_guess:=false
```

Rviz will open, and you will need to set the Fixed Frame as 'ground'.
Now you are able to see the atlas car model with the sensors in their first position.

Then, in a new terminal:
```
rosrun interactive_marker_test create_first_guess.py -w car_center
```

Now you can move the green markers and save the new sensors configuration.
Kill the booth process in the terminals, and run:

```
roslaunch interactive_marker_test rviz.launch car_model:=true read_first_guess:=true
```
Now you will see the atlas car model with the sensors in the updated position (don't forget to set Fixed Frame as 'ground').

## For PR2 robot model
For seeing the PR2 model instead of the AtlasCar2, just run the command (it's the same but with the car_model argument set as false)
```
roslaunch interactive_marker_test rviz.launch car_model:=false read_first_guess:=false
```
You need to set Fixed Frame as 'base_footprint' in order to see the urdf robot model.

# Known problems

## urdf model not showing on rviz or urdf model showed up misplaced

If you can't see the car on rviz or the model showed up on a wrong place, you
have to run this command before everything:

```
export LC_NUMERIC="en_US.UTF-8"
```

If this worked out, you must run always this command at the first place. To avoid this constant work, you can always
add this command to your .bashrc file (copy and paste at the end of the bashrc document).
Now you don't have to worry about this anymore!

# Recording a bag file of the ATLASCAR2

This should be echanced with the cameras.

```
rosbag record /left_laser/laserscan /right_laser/laser_scan
```
