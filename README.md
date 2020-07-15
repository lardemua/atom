# <img align="left" width="257" height="215" src="https://github.com/lardemua/atom/blob/master/docs/logo2.png?raw=true/514/431"> ATOM Calibration 
###### A Calibration Framework using the **A**tomic **T**ransformation **O**ptimization **M**ethod 

Atom is a set of calibration tools for multi-sensor, multi-modal, robotic systems. 

It is based on the optimization of atomic transformations as provided by a ros based robot description. 
Moreover, **ATOM** provides several scripts to facilitate all the steps of a calibration procedure. 

# Table of Contents

- [How to Use - Quick Start](#how-to-use---quick-start)
- [System calibration - Detailed Description](#system-calibration---detailed-description)
  * [Creating a calibration package](#creating-a-calibration-package)
  * [Configuring a calibration package](#configuring-a-calibration-package)
  * [Set initial estimate](#set-initial-estimate)
  * [Collect data](#collect-data)
  * [Calibrate sensors](#calibrate-sensors)
- [Contributors](#contributors)
- [Maintainers](#maintainers)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>


# How to Use - Quick Start

Unlike most other calibration approaches, **ATOM** offers tools to address the complete calibration pipeline:

0. **Prepare you environment** (OPTIONAL) - define environment variables for easy cross machine access to data
   ```bash
   export ROS_BAGS="$HOME/bagfiles"
   export ATOM_DATASETS="$HOME/datasets"
   ```
1. **Create a calibration package** for you robotic system
   ```bash
   rosrun atom_calibration create_calibration_pkg --name <your_robot_calibration>
   ```
2. **Configure your calibration package** - edit the file 
_<your_robot_calibration>/calibration/config.yml_ with your system information.
   ```bash
   rosrun <your_robot_calibration> configure 
   ```
3. **Set initial estimate** - deployment of interactive tools based on rviz that allow the user to set the pose of the sensors to be calibrated, while receiving visual feedback;
   ```bash
   roslaunch <your_robot_calibration> set_initial_estimate.launch 
   ```
4. **Collect Data** - Extraction of snapshots of data (a.k.a., collections)
   ```bash
   roslaunch <your_robot_calibration> collect_data.launch output_folder:=~/datasets/<my_dataset> 
   ```
   
5. **Calibrate sensors** - finally run an optimization that will calibrate your sensors:
```bash
roslaunch <your_robot_calibration> calibrate.launch dataset_file:=~/datasets/<my_dataset>/data_collected.json
```

# System calibration - Detailed Description

To calibrate your robot you must define your robotic system, (e.g. <your_robot>). You should also have a **system description** in the form of an urdf or a xacro file(s). This is normally stored in a ros package named **<your_robot>_description**. 

Finally, **ATOM** requires a bagfile with a recording of the data from the sensors you wish to calibrate. Transformations in the bagfile (i.e. topics /tf and /tf_static) will be ignored, so that they do not collide with the ones being published by the _robot_state_publisher_. Thus, if your robotic system contains moving parts, the bagfile should also record the _sensor_msgs/JointState_ message. 

It is also possible to record compressed images, since **ATOM** can decompress them while playing back the bagfile.

## Creating a calibration package

To start you should create a calibration ros package specific for your robot. **ATOM** provides a script for this:
```bash
rosrun atom_calibration create_calibration_pkg --name <your_robot_calibration>
```

This will create the ros package <your_robot_calibration> in the current folder, but you can also specify the folder, e.g.:
```bash
rosrun atom_calibration create_calibration_pkg --name ~/my/path/<your_robot_calibration>
```

## Configuring a calibration package

Once your calibration package is created you will have to configure the calibration procedure by editing the 
_<your_robot_calibration>/calibration/config.yml_ file with your system information. Here is an example of a [config.yml](templates/config.yml) file.

After filling the config.yml file, you can run the package configuration:

```bash
rosrun <your_robot_calibration> configure 
```

This will create a set of files for launching the system, configuring rviz, etc.

## Set initial estimate

Iterative optimization methods are often sensitive to the initial parameter configuration. Here, the optimization parameters represent the poses of each sensor. **ATOM** provides an interactive framework based on rviz which allows the user to set the pose of the sensors while having immediate visual feedback.

To set an initial estimate run:
```bash
roslaunch <your_robot_calibration> set_initial_estimate.launch 
```

Here are a couple of examples:

[Atlascar2](https://github.com/lardemua/atlascar2)  | [AgrobV2](https://github.com/aaguiar96/agrob)
------------- | -------------
<img align="center" src="https://github.com/lardemua/atom/blob/master/docs/set_initial_estimate_atlascar2.gif" width="450"/>  | <img align="center" src="https://github.com/lardemua/atom/blob/master/docs/set_initial_estimate_agrob.gif" width="450"/>

[UR10e eye in hand](https://github.com/iris-ua/iris_ur10e_calibration)  | ...
------------- | -------------
<img align="center" src="https://github.com/lardemua/atom/blob/master/docs/ur10e_eye_in_hand_set_initial_estimate.gif" width="450"/>  | ...

## Collect data 

To run a system calibration, one requires sensor data collected at different time instants. We refer to these as **data collections**. To collect data, the user should launch:
```bash
roslaunch <your_robot_calibration> collect_data.launch  output_folder:=<your_dataset_folder>
```

Depending on the size and number of topics in the bag file, it may be necessary (it often is) to reduce the playback rate of the bag file.
```bash
roslaunch <your_robot_calibration> collect_data.launch  output_folder:=<your_dataset_folder> bag_rate:=<playback_rate>
```

Here are some examples of the system collecting data:

[Atlascar2](https://github.com/lardemua/atlascar2)  | [AgrobV2](https://github.com/aaguiar96/agrob)
------------- | -------------
<img align="center" src="https://github.com/lardemua/atom/blob/master/docs/collect_data_atlascar2.gif" width="450"/>  | <img align="center" src="https://github.com/lardemua/atom/blob/master/docs/collect_data_agrob.gif" width="450"/>

... | [UR10e eye to_base](https://github.com/iris-ua/iris_ur10e_calibration)
------------- | -------------
... | <img align="center" src="https://github.com/lardemua/atom/blob/master/docs/ur10e_eye_to_base_collect_data.gif" width="450"/>  

A dataset is a folder which contains a set of collections. There, a _data_collected.json_ file stores all the information required for the calibration.

<img align="center" src="https://github.com/lardemua/atom/blob/master/docs/viewing_data_collected_json.gif" width="600"/> 

## Calibrate sensors

Finally, a system calibration is called through:

```bash
roslaunch <your_robot_calibration> calibrate.launch dataset_file:=~/datasets/<my_dataset>/data_collected.json
```

You can also define the following additional arguments:

```bash
single_pattern:=true
                    show a single pattern instead of one per collection.

use_incomplete_collections:=true
                    Remove any collection which does not have a detection
                    for all sensors.

-ssf "SENSOR_SELECTION_FUNCTION"
                    A string to be evaluated as a lambda function that
                    receives a sensor name as input and returns True or
                    False to indicate if the sensor should be loaded (and
                    used in the optimization). The Syntax is lambda name:
                    f(x), where f(x) is the function in python language.
                    Example: "lambda name: name in ["left_laser",
                    "frontal_camera"]" , to load only sensors left_laser
                    and frontal_camera

-csf "COLLECTION_SELECTION_FUNCTION"
                    A string to be evaluated into a lambda function that
                    receives a collection name as input and returns True
                    or False to indicate if the collection should be
                    loaded (and used in the optimization). The Syntax is
                    lambda name: f(x), where f(x) is the function in
                    python language. Example: "lambda name: int(name) > 5" ,
                    to load only collections 6, 7, etc .
```

# Contributors

 * Miguel Riem Oliveira - University of Aveiro
 * Afonso Castro - University of Aveiro
 * Eurico Pedrosa - University of Aveiro
 * Tiago Madeira - University of Aveiro
 * André Aguiar - INESC TEC

# Maintainers

 * Miguel Riem Oliveira - University of Aveiro
 * Eurico Pedrosa - University of Aveiro

