# ATOM_EXAMPLES

ATOM examples are a set of robotic systems which are used to exemplify the capabilities of ATOM. The examples cover several robotic systems, sensor modalities, and calibration configurations.

System |           Name   |   Description     | Pattern(s)
:---:|:---:|:---:|:---
 <img src="rgb_rgb_system/docs/system.png" width="100%"/> | [rgb_rgb_system](https://github.com/lardemua/atom/tree/noetic-devel/atom_examples/rgb_rgb_system) |Two rgb cameras mounted on a tripod. | <img src="../atom_worlds/pattern/models/charuco_800x600_5x5_100/texture.png" width="50%"/> <br>Charuco 800x600mm, moving around the scene.
 <img src="rgb_depth_system/docs/system.png" width="100%"/> | [rgb_depth_system](https://github.com/lardemua/atom/tree/noetic-devel/atom_examples/rgb_depth_system) |An RGBD camera, i.e., one rgb camera and a depth camera, both mounted on a tripod. | <img src="../atom_worlds/pattern/models/charuco_800x600_5x5_100/texture.png" width="50%"/> <br>Charuco 800x600mm, moving around the scene.
 <img src="rgb_rgb_system_chessboard/docs/system.png" width="100%"/> | [rgb_rgb_system_chessboard](https://github.com/lardemua/atom/tree/noetic-devel/atom_examples/rgb_rgb_system_chessboard) |Two rgb cameras mounted on a tripod. | <img src="../atom_worlds/pattern/models/chessboard_1200x900_10x7/texture.png" width="50%"/> <br>Chessboard 1200x900mm, moving around the scene.
 <img src="rgb_hand_eye_system/docs/system.png" width="100%"/> | [rgb_hand_eye_system](https://github.com/lardemua/atom/tree/noetic-devel/atom_examples/rgb_hand_eye_system) | An rgb camera mounted on the end-effector of a robotic manipulator. |<img src="../atom_worlds/pattern/models/charuco_800x600_5x5_100/texture.png" width="50%"/> <br>Charuco 800x600mm, static on top of table.
 <img src="eye_in_world_system/docs/system.png" width="100%"/> | [eye_in_world_system](https://drive.google.com/file/d/1uh4TjKMwsrK8L6OX9dyneeQFAPJDe1Yy/view?usp=drive_link) | An rgb camera mounted on a tripod, facing a robotic manipulator with a pattern attached to the end-effector. |<img src="../atom_worlds/pattern/models/charuco_200x200_4x4/texture.png" width="50%"/> <br>Charuco 200x200mm, static mounted on the end_effector.

# Download bagfiles and datasets

To download all the bagfiles and datasets required to run the atom examples, use the script:

    rosrun atom_calibration download_atom_examples_data

which puts the bagfiles in the folder defined in your $BAG_FILES environment variable. See the [ATOM documentation](https://lardemua.github.io/atom_documentation/getting_started/#set-environment-variables)

In the specific instructions for each example, the links to download bagfiles and datasets are given. However, **if you downloaded all atom examples data using the above instruction you can ignore those parts**.
