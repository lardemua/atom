# AtlasCarCalibration
Reading the markers starting position of an urdf robot sample and be able to move the interactive markers associated to them

# Usage Instructions
First , add this package to your catkin workspace.

Run roscore 
```
$ roscore
```

Then, in a new terminal:
```
$ roslaunch interactive_marker_test rviz.launch
```

Now rviz will open, and you will be able to see the robot with the red markers.

Then, in a new terminal:
```
$ rosrun interactive_marker_test interactive_market_test.py 
```

Now you can move the green markers and see the position (and orientation) of any of them at any time.

# Known problems

## urdf model not showing on rviz

See 

https://answers.ros.org/question/312706/rviz-does-not-show-3d-objects-from-urdf/
