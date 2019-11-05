# AtlasCarCalibration
Reading the sensors starting position of a robot xacro file (atlas car sample) and create interactive markers associated to them.
Saving the markers final position and reopen the robot with the sensors at the updated position.

# Table of Contents

- [AtlasCarCalibration](#atlascarcalibration)
- [Table of Contents](#table-of-contents)
- [Installation](#installation)
  * [Using PR2 robot instead of AtlasCar2](#using-pr2-robot-instead-of-atlascar2)
- [Usage Instructions](#usage-instructions)
  * [For PR2 robot model](#for-pr2-robot-model)
  * [Visualizing the calibration graphs](#visualizing-the-calibration-graphs)
- [Known problems](#known-problems)
  * [urdf model not showing on rviz or urdf model showed up misplaced](#urdf-model-not-showing-on-rviz-or-urdf-model-showed-up-misplaced)
- [Recording a bag file of the ATLASCAR2](#recording-a-bag-file-of-the-atlascar2)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>

# Installation

It will be needed some sensors packages.
In the same directory (catkin_ws/src):
```
git clone https://github.com/clearpathrobotics/LMS1xx.git
git clone https://github.com/ros-drivers/pointgrey_camera_driver.git
git clone https://github.com/SICKAG/sick_ldmrs_laser.git
git clone https://github.com/SICKAG/libsick_ldmrs.git
```
Also, in the same directory (catkin_ws/src), clone the atlas car model:
````
git clone https://github.com/lardemua/atlas-core.git
```` 

In order to compile this package, flycap is needed. To run FlyCapture2 on a Linux Ubuntu system, install the following dependencies:

```
sudo apt-get install libraw1394-11 libgtkmm-2.4-1v5 libglademm-2.4-1v5 libgtkglextmm-x11-1.2-dev libgtkglextmm-x11-1.2 libusb-1.0-0
```
And then:
```
sudo sh install_flycapture.sh
```
(See if flycap works. It probably need some updates as well)

Finally, you will need colorama:
```
sudo pip install colorama
```
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
roslaunch interactive_calibration atlascar2_calibration.launch
```

Rviz will open. It is better if you check the Fixed Frame: it must be 'base_link'. 
You must also add the location and name of the file that will store the first guess data with the -f argument. 
Location is given starting from the path of the interactive_calibration ros package.
now you are able to see the atlas car model with the sensors in their first position.

Then, in a new terminal:
```
rosrun interactive_calibration create_first_guess.py -w base_link -f /calibrations/atlascar2/first_guess.urdf.xacro
```

Now you can move the green markers and save the new sensors configuration.
Kill the booth process in the terminals, and run:

```
roslaunch interactive_calibration atlascar2_calibration.launch read_first_guess:=true
```
Now you will see the atlas car model with the sensors in the updated position (don't forget: Fixed Frame should be 'base_link').

## For PR2 robot model
For seeing the PR2 model instead of the AtlasCar2, just run the command (it's the same but with the car_model argument set as false)
```
roslaunch interactive_calibration rviz.launch car_model:=false read_first_guess:=false
```
You need to set Fixed Frame as 'base_footprint' in order to see the urdf robot model.

## Visualizing the calibration graphs

In order to visualize the calibration graphs you may run:

```
rosrun interactive_calibration draw_calibration_graph.py -w {world_frame}
```

Annotated tf trees are displayed to better understand the calibration process. Here are some examples for the PR2 robot:

![calibration_full](https://github.com/lardemua/AtlasCarCalibration/blob/master/docs/calibration_full.png) 

![calibration_per_sensor](https://github.com/lardemua/AtlasCarCalibration/blob/master/docs/calibration_per_sensor.png) 

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
# Run cos optimization first test

```
rosrun interactive_calibration first_optimization.py
```

If it not works, run first 

```
chmod +xfirst_optimization.py
```
