# SOFTBot
**S**ensors to **O**dom **F**rame **T**est ro**B**ot (softbot) is a conceptual robot designed to test advanced calibration methodologies for mobile robotic platforms. In particular, this platform is focused on the calibration of sensors such as rgb cameras or LiDARs w.r.t. the coordination frame of the motion model of the robot. 

We refer to this functional coordinate frame, around which the robot rotates as it moves, as the **odom frame**, since it is used to compute the odometry. Naturally, the odom frame changes position in accordance with the motion model of the robot, e.g. differential drive, ackerman steering etc. 

The system contains the following sensors:
- **3dlidar** - A 3D LiDAR mounted on the robot frame;
- **rgbd_cameras** - Two RGB-D cameras mounted on the robot frame, two facing the front of the robot.



![softbot_gazebo](docs/gazebo_softbot.png)

and then you can also view it in rviz:

<!-- ![softbot_gazebo](docs/softbot_rviz_updated.gif)  -->
<img src="docs/softbot_rviz_updated.gif" width="1080px" />


# Installation

##### Add to .bashrc:
```
export GAZEBO_MODEL_PATH="`rospack find softbot_gazebo`/models:${GAZEBO_MODEL_PATH}"
```
# How to run

First launch the gazebo simulation:

    roslaunch softbot_gazebo softbot.launch 

Then you can spawn the softbot robot:

    roslaunch softbot_bringup bringup.launch

If you have a controller and desire to use it to maneuver the robot you can :

    roslaunch softbot_bringup bringup.launch controller:=true max_controller_vel:=0.2

-   **R2** -> Throttle
-   **L2** -> Break
-   **Left Joystick** -> Steer
  
  The user should be careful that any velocity above **0.2** causes some serious drifting and instability.


After launching the entire system you can record a bagfile to use later in the calibration pipeline :

    roslaunch softbot_bringup record.launch

**OBS:**
       
The collection is automaticaly saved in `/tmp`: 
````
tmp_<yyyy-mm-dd-hh-mm-ss>.bag
````


#   Configuring a calibration package
Once your calibration package is created you will have to configure the calibration procedure by editing the *softbot_calibration/calibration/config.yml* file with your system information.
    
    rosrun softbot_calibration configure 

This will create a set of files for launching the system, configuring rviz, etc. To accomplish this, ATOM reads the data from the given bagfile and robot xacro description, meaning if you ought to use another bagfile, you **should** tweak the *config.yml* and execute the configure script **again**.

After configuring, the user's transformation summary should look similar to this :

![softbot_gazebo](docs/summary.jpg)


After configuring, the user can play the bag file:

    roslaunch softbot_calibration playbag.launch

Here is a pre-recorded bagfile should the user decide to skip the recording and proceed to the calibration itself. 

**PLACEHOLDER FOR BAGFILE LINK
E
E
E
E
E
E**

**Note :**
Previously ATOM wasn't prepared to lead with transformations not defined in the xacro seamlessly, as is the case here in the transformation from the odom's frame to the robot's `base_footprint` hence it was required to configure with a now depracated flag `-utf`, which would discard the transformations from the xacro and use exclusively the data from the bagfile. This approach had the big downside that the user would lost all joint related info such as the type of joint. Now ATOM leads with this technicality seamlessly, the user doesn't have to use anything.

In softbot's case this will happen, since the transformation from `world` to `base_footprint` is dynamic and not defined in the xacro. Thus *ATOM* will create a auxiliary `<bagfile_name>_filtered.bag` containing only the transformations defined in the previous phrase. *Atom* then reads this bagfile and publishes all the transforms found here in `/tf`.



#   Collect data

To run a system calibration, one requires sensor data collected at different time instants. We refer to these as data collections. To collect data, the user should launch:

    roslaunch softbot_calibration collect_data.launch  output_folder:=~/datasets/softbot/dataset1 overwrite:=true

Whenever there are LiDARs in the equation, it is almost necessary for the user to correct the automatic labeling. For doing so the user should run :

    roslaunch softbot_calibration dataset_playback.launch

... and ...

    rosrun atom_calibration dataset_playback -json $ATOM_DATASETS/softbot/dataset1/dataset_corrected.json


<!-- ![softbot_gazebo](docs/dataset_labeling.png) -->
![softbot_gazebo](docs/dataset_labeling_noGUI.png)

The user's goal is to make sure the **only** green points being those of the pattern, distinguishing between **dark** green for border points and **light** green inside the pattern.

More information regarding this topic can be found in [*Atom*'s wiki](https://lardemua.github.io/atom_documentation/procedures/#3d-lidar-labeling).

Here is a pre-labeled dataset should the user decide to skip the recording and proceed to the calibration itself. 

**PLACEHOLDER FOR TRAIN DATASET LINK
E
E
E
E
E
E**

Collection |           front_left_camera        | front_right_camera            |           lidar3d/points
:---------:|:----------------------------------:|:-----------------------------:|:------------------------------:
0          | ![](docs/dataset_imgs/0L.png)      | ![](docs/dataset_imgs/0R.png) | <img src="docs/dataset_imgs/0pcd.png" heigth="720px" />
1          | ![](docs/dataset_imgs/1L.png)      | ![](docs/dataset_imgs/1R.png) | <img src="docs/dataset_imgs/1pcd.png" heigth="720px" />
2          | ![](docs/dataset_imgs/2L.png)      | ![](docs/dataset_imgs/2R.png) | <img src="docs/dataset_imgs/2pcd.png" heigth="720px" />
3          | ![](docs/dataset_imgs/3L.png)      | ![](docs/dataset_imgs/3R.png) | <img src="docs/dataset_imgs/3pcd.png" heigth="720px" />

**NOTE :**
In order to select the points, the user should use the "*SelectedPointsPublisher*" tool in rviz. In case this tool isn't by default on the toolbar, the use can add it by pressing the **+** button on the far right of the toolbar.

# Calibrate sensors

Finally the user can run an optimization that will calibrate your sensors:

    roslaunch softbot_calibration calibrate.launch


and then launch the calibration script:

    rosrun atom_calibration calibrate \
    -json $ATOM_DATASETS/softbot/dataset1/dataset_corrected.json \
    -v -rv -si -csf "lambda x : int(x)" -ipg -nig 0.02 0.02

- -json     -> Path to the dataset **json**
- -v        -> **V**erbose mode
- -rv       -> Enable **R**os **V**isualization
- -si       -> **S**how **I**mages in Rviz
- -csf      -> **C**ollection **S**election **F**unction
- -ipg      -> **I**nitial **P**osition **G**host
- -nig  x y -> **N**oisy **I**nitial **G**uess x[cm] in translation, y[º] in rotation



By following this example, the user's end output should be something similar with :

```` 
+------------+------------------------+-------------------------+-------------+
| Collection | front_left_camera [px] | front_right_camera [px] | lidar3d [m] |
+------------+------------------------+-------------------------+-------------+
|    001     |         1.1104         |          0.8573         |    0.0059   |
|    002     |         0.9173         |          0.7258         |    0.0068   |
|    003     |         0.7595         |          0.5824         |    0.0067   |
|    004     |         0.4805         |          0.4594         |    0.0053   |
|  Averages  |         0.8169         |          0.6562         |    0.0062   |
+------------+------------------------+-------------------------+-------------+
````

And by comparing with the **initial position ghost** (-ipg), the user can visually conclude about the calibration :


<!-- ![softbot_gazebo](docs/calibration_results.png) -->

<img src="docs/ipg.gif" width="540px" />

## Evaluation

Before anything, the user must annotate the pattern in the RGB images : (To do so it's necessary a test dataset, which the user can record or download here **PLACEHOLDER FOR Test DATASET LINK
E
E
E
E
E
E**
)

    rosrun atom_evaluation annotate_pattern_borders_in_rgb_or_depth \
    --dataset $ATOM_DATASETS/softbot/test_dataset/dataset_corrected.json \
    --rgb_sensor front_right_camera


...and...

    rosrun atom_evaluation annotate_pattern_borders_in_rgb_or_depth \
    --dataset /home/brunofavs/datasets/softbot/test_dataset/dataset_corrected.json\
     --rgb_sensor front_left_camera

Should look similar to this for every collection and camera :

![softbot_gazebo](docs/annotation.png)


To evaluate the calibration that was done, the user can run :

    roslaunch softbot_calibration full_evaluation.launch \
    train_json:=$ATOM_DATASETS/softbot/dataset_perfect_simulation/dataset_corrected.json \
    test_json:=$ATOM_DATASETS/softbot/test_dataset/dataset_corrected.json 

And for example the results per collection are :

```
+--------------+-----------+-------------+-------------+------------+-----------+
| Collection # | RMS (pix) | X err (pix) | Y err (pix) | Trans (mm) | Rot (deg) |
+--------------+-----------+-------------+-------------+------------+-----------+
|     000      |   3.7431  |    1.1038   |    3.4902   |   9.5851   |   0.0416  |
|     001      |   3.1000  |    0.7625   |    2.9246   |   9.5074   |   0.0489  |
|     002      |   3.7925  |    1.3990   |    3.4694   |  10.1129   |   0.0466  |
|     003      |   3.7405  |    1.0998   |    3.4892   |   9.5648   |   0.0426  |
|     004      |   3.0920  |    0.7469   |    2.9123   |   9.2147   |   0.0111  |
|   Averages   |   3.4936  |    1.0224   |    3.2571   |   9.5970   |   0.0382  |
+--------------+-----------+-------------+-------------+------------+-----------+
```

...and per transformation are :

```

+--------------+-----------+-------------+-------------+------------+-----------+
| Collection # | RMS (pix) | X err (pix) | Y err (pix) | Trans (mm) | Rot (deg) |
+--------------+-----------+-------------+-------------+------------+-----------+
|     000      |   3.7431  |    1.1038   |    3.4902   |   9.5851   |   0.0416  |
|     001      |   3.1000  |    0.7625   |    2.9246   |   9.5074   |   0.0489  |
|     002      |   3.7925  |    1.3990   |    3.4694   |  10.1129   |   0.0466  |
|     003      |   3.7405  |    1.0998   |    3.4892   |   9.5648   |   0.0426  |
|     004      |   3.0920  |    0.7469   |    2.9123   |   9.2147   |   0.0111  |
|   Averages   |   3.4936  |    1.0224   |    3.2571   |   9.5970   |   0.0382  |
+--------------+-----------+-------------+-------------+------------+-----------+
```