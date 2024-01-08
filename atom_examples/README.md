# ATOM_EXAMPLES

ATOM examples are a set of robotic systems which are used to exemplify the capabilities of ATOM. The examples cover several robotic systems, sensor modalities, and calibration configurations.

System |           Name   |   Description
:---:|:---:|:---:
<img src="rrbot/docs/table.png" width="4000"/>  | [rrbot](rrbot)| Calibration of two RGB sensors.
 <img src="rdbot/docs/table.png" width="100%"/> | [rdbot](https://github.com/lardemua/atom/tree/noetic-devel/atom_examples/rdbot) | Calibration of an RGB-D sensor.
 <img src="rrcbot/docs/table.png" width="100%"/> | [rrcbot](https://github.com/lardemua/atom/tree/noetic-devel/atom_examples/rrcbot) | Calibration using chessboard patterns.
 <img src="rihbot/docs/table.png" width="100%"/> | [rihbot](https://github.com/lardemua/atom/tree/noetic-devel/atom_examples/rihbot) | Hand-eye calibration, eye-in-hand variant.
 <img src="riwbot/docs/table.png" width="100%"/> | [riwbot](https://github.com/lardemua/atom/tree/noetic-devel/atom_examples/riwbot) | Hand-eye calibration, eye-in-world variant.
<img src="mrjbot/docs/table.png" width="100%"/> | [mrjbot](https://github.com/lardemua/atom/tree/noetic-devel/atom_examples/mrjbot) |  Calibration of revolute joints.
<img src="spjbot/docs/table.png" width="100%"/> | [spjbot](https://github.com/lardemua/atom/tree/noetic-devel/atom_examples/spjbot) | Calibration of prismatic joints.
<img src="rihmpbot/docs/table.png" width="100%"/> | [rihmpbot](https://github.com/lardemua/atom/tree/noetic-devel/atom_examples/rihmpbot) | Using multiple calibration patterns.
 <img src="riwmpbot/docs/table.png" width="100%"/> | [riwmpbot](https://github.com/lardemua/atom/tree/noetic-devel/atom_examples/riwbot) | Improving calibration with multiple patterns.


<!-- Old name | Full new name | acronym
:---:|:---:|:---:
------- | rgb_in_hand_joint_optimization_robot | rihjobot
------- | rgb_in_world_joint_optimization_robot | riwjobot
------- | lidar_robot | lbot
------- | rgb_rgb_multiple_patterns_robot | rmpbot -->

# Download bagfiles and datasets

To download all the bagfiles and datasets required to run the atom examples, use the script:

    rosrun atom_calibration download_atom_examples_data

which puts the bagfiles in the folder defined in your $BAG_FILES environment variable. See the [ATOM documentation](https://lardemua.github.io/atom_documentation/getting_started/#set-environment-variables)

In the specific instructions for each example, the links to download bagfiles and datasets are given. However, **if you downloaded all atom examples data using the above instruction you can ignore those parts**.
