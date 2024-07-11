# Dataset Reviewer

Tool to manually label 3D point clouds.

Launch rviz preconfigured:

    roslaunch <robot_name> dataset_playback.launch

Launch dataset_reviewer script:

    rosrun atom_calibration dataset_playback2 -json $ATOM_DATASETS/<robot_name>/<path_to_dataset>/dataset.json -uic -si

Then, select the SelectedPointsPublisher plugin. 

Once the plugin is selected, click on the points you want and press:

- **p** to add those points to 'idxs'
- **b** to add those points to 'idxs_limit_points'
- **r** to remove those points from 'idxs' and/or 'idxs_limit_points'
- **c** to remove all points from both 'idxs' and 'idxs_limit_points'

To navigate across collections, use the **right** and **left** arrows.

If you want to label multiple point clouds, it is advisable to only visualize the one you are currently labeling.

To save the corrected dataset, just select the terminal that launched the dataset_reviewer and press Ctr+C followed by yes.