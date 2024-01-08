<!-- # RGB RGB SYSTEM CHESSBOARD -->

The **R**GB **R**GB with **c**hessboard ro**bot** (**rrcbot**) is a robotic system meant to be use in simple tests and to serve as example for ATOM beginners. The novelty here is that the system uses a chessboard as calibration pattern, instead of the usual charuco board.

![gazebo](docs/system.png)

The system is composed of two rgb cameras mounted on a tripod.
Cameras are called **rgb_left** (red) and **rgb_right** (green).
The system contains the following topics:

  - /rgb_left/camera_info
  - /rgb_left/image_raw
  - /rgb_right/camera_info
  - /rgb_right/image_raw
  - /tf
  - /tf_static

Since this is a systems to test calibration, where frame rate is not a critical issue, we restrained images topics to 10Hz.
This is a simulated system, which can be seen in gazebo:

![gazebo](docs/gazebo.png)

... and in rviz:

![rviz](docs/rviz.png)

# How to run

First launch the gazebo simulation:

    roslaunch rrcbot_gazebo gazebo.launch

Then you can bringup the system:

    roslaunch rrcbot_bringup bringup.launch

You can record a bag file using:

    roslaunch rrcbot_bringup record.launch

This will put the bag file into your $ROS_BAGS folder.

# Calibration

The calibration of any robotic system using **ATOM** may have several variants. We recommend a careful reading of the [documentation](https://lardemua.github.io/atom_documentation/) to learn all the details.

In this section, out goal is to carry out the simplest possible calibration pipeline for the **rrcbot**.

To calibrate, we will need a bagfile called [train.bag](https://drive.google.com/file/d/1zucjVjg27plyvN-vWpAyo3-utY7Xro--/view?usp=sharing), which contains a recording of the system's data when viewing a calibration pattern in several positions.
We produced the bagfile by bringing up the system and then recording a bagfile as described above.
This is a small bagfile with 50 seconds / 55MB for demonstration purposes. Typically, calibration bagfiles are larger.

Download the bagfile and put it in **$ROS_BAGS/rrcbot**.

Next we describe each of the steps in the calibration pipeline.

## Creating a calibration package

See also the [generic documentation](https://lardemua.github.io/atom_documentation/procedures/#create-a-calibration-package) on this topic.

Using ATOM conventions, we define name of the calibration package as **rrcbot_calibration**, and create it using:

    rosrun atom_calibration create_calibration_pkg --name rrcbot_calibration

**NOTE**: This procedure is carried out only once, and was done already. As such, this ros package is already [included in the atom repo](https://github.com/lardemua/atom/tree/noetic-devel/atom_examples/rrcbot/rrcbot_calibration). Therefore, you **should not execute this instruction** for the rrcbot.


## Configuring the calibration

This is the [config.yml](https://github.com/lardemua/atom/blob/noetic-devel/atom_examples/rrcbot/rrcbot_calibration/calibration/config.yml) that we wrote to define the calibration. There are two sensors to be calibrated, named **rgb_left** and **rgb_right**. The pattern is a **chessboard marker**.
The configuration file points to the bagfile mentioned above, and the _anchored_sensor_ is defined as the **rgb_left** sensor.

To configure run:

    rosrun rrcbot_calibration configure

Which will run a series of checks and produce several files inside the **rrcbot_calibration** package.


## Collecting a dataset

To collect a dataset we run:

    roslaunch rrcbot_calibration collect_data.launch output_folder:=$ATOM_DATASETS/rrcbot/dataset1 overwrite:=true

And save a few collections.

We will use as example the [train](https://drive.google.com/file/d/1FOd4HHPMB0Yn7lCSDMu2wveZW3I-k6QM/view?usp=sharing) dataset, which contains 6 collections, as shown bellow.

Download and decompress the dataset to **$ATOM_DATASETS/rrcbot/train**.

Collection |           rgb_left             |           rgb_right
:----------------:|:-------------------------:|:-------------------------:
0 | ![](docs/rgb_left_000.jpg) |  ![](docs/rgb_right_000.jpg)
1 | ![](docs/rgb_left_001.jpg) |  ![](docs/rgb_right_001.jpg)
2 | ![](docs/rgb_left_002.jpg) |  ![](docs/rgb_right_002.jpg)
3 | ![](docs/rgb_left_003.jpg) |  ![](docs/rgb_right_003.jpg)
4 | ![](docs/rgb_left_004.jpg) |  ![](docs/rgb_right_004.jpg)
5 | ![](docs/rgb_left_005.jpg) |  ![](docs/rgb_right_005.jpg)


## Running the Calibration

To calibrate, first setup visualization with:

    roslaunch rrcbot_calibration calibrate.launch

Then carry out the actual calibration using, using a **nig** to give some noise in the initial estimate:

    rosrun atom_calibration calibrate -json $ATOM_DATASETS/rrcbot/train/dataset.json -v -rv -si -nig 0.1 0.1

At the beginning of the calibration, the error table is this:

![](docs/output_start.png)

where we can see that the errors for the **rgb_right** camera are very high due to the noise that was induced in the initial estimate. The initial state of the system is the following:

![](docs/optimization_start.png)

where it is possible to see that the **rgb_right** camera is below the support bar of the tripod, i.e., not in the correct position.

After the optimization finishes we get:

![](docs/output_end.png)

Here we can see that as a result of the calibration, the errors are much lower.
This is also observed visually:

![](docs/optimization_end.png)

since the position of the **rgb_right** is now on top of the support bar.

## Evaluation

The evaluation be conducted with a second dataset which has not been seen during calibration. We call these the test datasets.

Download the [test](https://drive.google.com/file/d/1bvHr2iXsNS3snF9Af8gJ0mJJbPwUcTpJ/view?usp=sharing) dataset and decompress to **$ATOM_DATASETS/rrcbot/test**.
Then run:

    roslaunch rrcbot_calibration full_evaluation.launch test_json:=$ATOM_DATASETS/rrcbot/test/dataset.json train_json:=$ATOM_DATASETS/rrcbot/train/atom_calibration.json

which will produce a table comparing the estimated pose versus the ground truth pose, which is only possible to compute because we are using a simulated system:

![](docs/ground_truth_errors.png)

which are very small, meaning the calibration is very accurate.

It is also possible to see a table comparing the sensor to sensor projections. This is interesting since it is possible to have also with real data:


![](docs/rgb_rgb_evaluation.png)

Also in this evaluation, the errors are very low, which means the calibration is very accurate.