# AtlasCarCalibration
Reading the markers starting position of an urdf robot sample and be able to move the interactive markers associated to them

# Usage Instructions
First , add this package to your catkin workspace.

Run roscore 
```
roscore
```

Then, in a new terminal:
```
roslaunch interactive_marker_test rviz.launch
```

Now rviz will open, and you will be able to see the robot with the red markers.

Then, in a new terminal:
```
rosrun interactive_marker_test interactive_market_test.py 
```

Now you can move the green markers and see the position (and orientation) of any of them at any time.

# Known problems

## urdf model not showing on rviz

If you can't see the robot on rviz, run this command before everything:

```
export LC_NUMERIC="en_US.UTF-8"
```

If this worked out, you must run always this command at the first place. To avoid this constant work, you can always
add this command to your bashrc file (copy and paste at the end of the bashrc document).
Now you don't have to worry about this anymore!

# TODO

- [ ] Listen the broadcaster (/robot_state_publisher) to get the position of the sensors

- [ ] Create interactive markers on that positions

- [ ] Publish the pose of all interactive markers when they move

- [ ] Listen de new pose of all interactive markers
