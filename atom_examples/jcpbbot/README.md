# JCPBBOT

The **j**oint **c**alibration **p**osition **b**ias (**jcpbbot**) is a robotic system used to exemplify joint calibrations in ATOM.
The goal of this system is to show how ATOM may be configured to calibrate the joints in a robotic system.
In this case, only the position bias parameter of the joints is considered. These are the errors produced by offsets in the rotational or linear encoders.

![gazebo](docs/system.png)

The configuration of the system is exactly the same as the [rihbot](https://github.com/lardemua/atom/tree/noetic-devel/atom_examples/rihbot), including the elements in the scene, the positioning of sensors, and the ros topics, which is why we will not specify these here.

# How to run

First launch the gazebo simulation:

    roslaunch jcpbbot_gazebo gazebo.launch

Then you can bringup the system:

    roslaunch jcpbbot_bringup bringup.launch

You can record a bag file using:

    roslaunch jcpbbot_bringup record.launch

This will put the bag file into your \$ROS_BAGS folder. You should move it to the **$ROS_BAGS/jcpb** folder.

# Calibration

As always, we recommend a careful reading of the [documentation](https://lardemua.github.io/atom_documentation/) to learn all the details.

In this section, out goal is to describe the calibration pipeline for the **jcpbbot**.

To calibrate, we will need a bagfile called [train.bag](https://drive.google.com/file/d/1trvpsJ9W5R0UkSHaOohmr4BZvnXY6ly0/view?usp=drive_link), which contains a recording of the system's data when viewing a calibration pattern in several positions.

Download the bagfile and put it in **$ROS_BAGS/jcpbbot/train.bag**.

## Adding a bias to the joints in the bagfile

The instructions in [How to Run](#how-to-run) will produce a bagfile that will contain joint values in the **/joint_states** message topic. Since these are produced by a simulation, the values will be perfect. To test the calibration of joint bias, we need to insert some bias to disrupt those perfect measurements, using the following command:

    rosrun atom_calibration add_noise_to_joint_state_in_bag -bfi $ROS_BAGS/jcpbbot/train.bag -bfo $ROS_BAGS/jcpbbot/train_with_noise.bag -jn shoulder_lift_joint elbow_joint wrist_1_joint wrist_2_joint wrist_3_joint shoulder_pan_joint -jb 0.034 -0.03 0.05 0.01 -0.03 -0.01

which means we are introducing a bias in the joints values of non-neglectable magnitude, which should present a challenge for the calibration.

You can download [train_with_noise.bag](https://drive.google.com/file/d/19xjwTXsZkcx5NNL_K3OPWv_Dv6El9ym0/view?usp=sharing) if you wish to skip this step.

We can now move forward to the configuration of the calibration package.

## Creating a calibration package

Using ATOM conventions, we define name of the calibration package as **jcpbbot_calibration**, and create it using:

    rosrun atom_calibration create_calibration_pkg --name jcpbbot_calibration

**NOTE**: This procedure is carried out only once, and was done already. As such, this ros package is already [included in the atom repo](https://github.com/lardemua/atom/tree/noetic-devel/atom_examples/jcpbbot/jcpbbot_calibration). Therefore, you **should not execute this instruction** for the jcpbbot.

## Configuring the calibration

This is the [config.yml](https://github.com/lardemua/atom/blob/noetic-devel/atom_examples/jcpbbot/jcpbbot_calibration/calibration/config.yml) that we wrote to define the calibration. There is a single sensor to be calibrated, named **rgb_hand**. The pattern is a charuco marker. Notice that we are using the **train_with_noise.bag** bagfile.


To configure run:

    rosrun jcpbbot_calibration configure

Which will run a series of checks and produce several files inside the **jcpbbot_calibration** package.

The configuration produces a [visual schematic summarizing](https://github.com/lardemua/atom/blob/noetic-devel/atom_examples/jcpbbot/jcpbbot_calibration/calibration/summary.pdf) the calibration you have setup.

![](docs/summary.png)

As we can see the calibration configuration will estimate the parameters of a complete static transformation, from **flange** to **rgb_hand_link**. These are the parameters that will position the rgb_hand sensor w.r.t. the end effector of the robotic manipulator. This component is a classical eye-in-hand calibration, as discussed in the [rihbot](https://github.com/lardemua/atom/tree/noetic-devel/atom_examples/rihbot) example.

The additional complexity comes from the calibration of the **position_bias** parameters of all of the manipulator's revolute joints, which is also visible in the image.

Finally, the system will use a single calibration pattern, a charuco which is static in the scene.

It is advisable to inspect this document carefully to make sure that the calibration is well configured.

## Collecting a dataset

To collect a dataset we run:

    roslaunch jcpbbot_calibration collect_data.launch output_folder:=$ATOM_DATASETS/jcpbbot/dataset1 overwrite:=true

And save a few collections.

We will use as example the [train](https://drive.google.com/file/d/1WjbzB9MRPmGcowggLKX-zDOaKnj89yRF/view?usp=sharing) dataset, which was created using the train_with_noise bagfile and thus contains joint position errors. The dataset contains 11 collections. Some are shown below.

Download and decompress the dataset to **$ATOM_DATASETS/jcpbbot/train**.

Collection |           rgb_hand
:----------------:|:-------------------------:
0 | ![](docs/rgb_hand_000.jpg)
1 | ![](docs/rgb_hand_001.jpg)
7 | ![](docs/rgb_hand_007.jpg)
8 | ![](docs/rgb_hand_008.jpg)


## Running the Calibration

To calibrate, first setup visualization with:

    roslaunch jcpbbot_calibration calibrate.launch

This is useful to visualize the collections stored in the dataset. Here you can see a visual representation of all collections and the corresponding camera at the start of the optimization.

![gazebo](docs/calibration.png)


Carry out the actual calibration including noise, using:

    rosrun atom_calibration calibrate -json $ATOM_DATASETS/jcpbbot/train/dataset.json -v -rv -nig 0.1 0.1 --phased -pp

The **--phased** flag will make the optimization halt before starting to change the parameters. With it it is possible to see the visual representation of the initial values of the parameters.
Because we used the nig flag, zooming into a single collection shows the camera is misplaced.

![gazebo](docs/before.png)

After calibration, the script will produce a table of residuals per iteration which starts the calibration with these errors:

![](docs/calibration_output_initial.png)

which are quite high, because of the incorrect pose of the sensors,  and ends up converging into these figures:

![](docs/calibration_output_final.png)

Which shows subpixel accuracy. This means the procedure achieved a successful calibration.

After calibration, the same collection shows the camera in place:

![gazebo](docs/after.png)

Moreover, because we used the **--print_parameters** (-pp) flag, the script will print, at the end of the optimization, the estimated values of the calibrated parameters.
The following table shows the estimated parameters vs the imposed biases.

Joint name | Inserted bias (rad) | Estimated bias (rad) | error (rad) | error (deg)
:---:|:---:|:---:|:---:|:---:
shoulder_lift_joint | 0.034 | -0.031488 | 0.002512 | 0.144
elbow_joint | -0.03 |0.029642 | 0.001358 | 0.078
wrist_1_joint | 0.05 |-0.050033 |0.000033 |0.001
wrist_2_joint | 0.01 |-0.010048 |0.000048 | 0.003
wrist_3_joint | -0.03 |0.019378 | 0.010622 |0.609
shoulder_pan_joint |-0.01 |0.009886 |0.000114 |0.007

This shows that the optimization was able to compensate for the errors introduced in the joints, by estimating compensation bias which are very close to corresponding induced errors.
The largest difference between the induced error and the estimated bias occurs in case of the wrist_3_joint, and has a magnitude of just 0.6 degrees.

## Evaluation

...
