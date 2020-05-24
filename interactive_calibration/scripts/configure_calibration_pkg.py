#!/usr/bin/env python

# ------------------------
#    IMPORT MODULES      #
# ------------------------
import argparse
import os
import rospkg
import subprocess
from copy import deepcopy
from datetime import date, datetime

import yaml
from colorama import Style, Fore

import rosbag
import rospy

from urdf_parser_py.urdf import URDF

# Add diplays as a function of the sensors used
display_dicts = {
    'rviz/Image': {'Name': 'TopCRGBDCamera-Image', 'Min Value': 0, 'Enabled': False, 'Value': False,
                   'Transport Hint': 'raw',
                   'Image Topic': '/top_center_rgbd_camera/rgb/image_raw', 'Queue Size': 2, 'Max Value': 1,
                   'Unreliable': False,
                   'Median window': 5, 'Class': 'rviz/Image', 'Normalize Range': True},
    'rviz/LaserScan': {'Min Color': '0; 0; 0', 'Style': 'Points', 'Use rainbow': True, 'Name': 'Left-LaserScan',
                       'Autocompute Intensity Bounds': True, 'Enabled': True, 'Value': True,
                       'Autocompute Value Bounds': {'Max Value': 0.5, 'Min Value': 0.5, 'Value': True},
                       'Size (m)': 0.10000000149011612, 'Unreliable': False, 'Color Transformer': 'FlatColor',
                       'Decay Time': 0, 'Size (Pixels)': 5, 'Min Intensity': 215, 'Use Fixed Frame': True,
                       'Max Intensity': 695, 'Color': '239; 41; 41', 'Invert Rainbow': False,
                       'Topic': '/left_laser/laserscan', 'Max Color': '255; 255; 255', 'Channel Name': 'intensity',
                       'Queue Size': 10, 'Position Transformer': 'XYZ', 'Alpha': 1, 'Selectable': True,
                       'Class': 'rviz/LaserScan', 'Axis': 'Z'},
    'rviz/PointCloud2': {'Min Color': '0; 0; 0', 'Style': 'Points', 'Use rainbow': True,
                         'Name': 'TopCRGBD-PointCloud2',
                         'Autocompute Intensity Bounds': True, 'Enabled': False, 'Value': False,
                         'Autocompute Value Bounds': {'Max Value': 9.872922897338867,
                                                      'Min Value': 0.9480007290840149, 'Value': True},
                         'Size (m)': 0.009999999776482582, 'Unreliable': False, 'Color Transformer': 'AxisColor',
                         'Decay Time': 0,
                         'Size (Pixels)': 3, 'Min Intensity': 0, 'Use Fixed Frame': False, 'Max Intensity': 4096,
                         'Color': '241; 225; 37', 'Invert Rainbow': False,
                         'Topic': '/top_center_rgbd_camera/depth/points',
                         'Max Color': '255; 255; 255', 'Channel Name': 'intensity', 'Queue Size': 10,
                         'Position Transformer': 'XYZ',
                         'Alpha': 1, 'Selectable': True, 'Class': 'rviz/PointCloud2', 'Axis': 'Z'},
    'rviz/Camera': {'Image Rendering': 'background and overlay', 'Name': 'TopRight-Camera', 'Enabled': False,
                    'Value': False,
                    'Overlay Alpha': 0.5, 'Transport Hint': 'raw', 'Zoom Factor': 1, 'Queue Size': 2,
                    'Visibility': {'TopCRGBDCamera-Image': True, 'TopRight-Image': True, 'TopLeft-Camera': True,
                                   'Left-LaserScan': True, 'FrontalLaser-PointCloud2': True, 'Value': True,
                                   'DataLabeler-InteractiveMarkers': False, 'RightLaser-Labels': False,
                                   'Right-LaserScan': True,
                                   'Camera': True, 'RightLaser-Clusters': True, 'LeftLaser-Clusters': True,
                                   'TopCRGBDCam-Image-Labelled': True, 'TopLeft-Image': True,
                                   'LeftLaser-Labels': False,
                                   'TopLeftImage-Labeled': True, 'FirstGuess-InteractiveMarkers': True,
                                   'RobotModel': False,
                                   'TopRightImage-Labeled': True, 'Grid': True, 'TopCRGBD-PointCloud2': True,
                                   'TF': False},
                    'Unreliable': False, 'Class': 'rviz/Camera', 'Image Topic': '/top_right_camera/image_color'},
    'rviz/InteractiveMarkers': {'Show Visual Aids': False, 'Name': 'DataLabeler-InteractiveMarkers',
                                'Update Topic': '/data_labeler/update', 'Enabled': True, 'Show Descriptions': False,
                                'Value': True, 'Show Axes': False, 'Enable Transparency': True,
                                'Class': 'rviz/InteractiveMarkers'},
    'rviz/tf': {'Frame Timeout': 15, 'Name': 'TF', 'Enabled': False, 'Marker Scale': 1, 'Tree': {}, 'Value': False,
                'Show Axes': True, 'Update Interval': 0, 'Show Names': False, 'Show Arrows': False,
                'Frames': {'All Enabled': True}, 'Class': 'rviz/TF'},
    'rviz/grid': {'Reference Frame': 'base_footprint', 'Name': 'Grid', 'Cell Size': 1, 'Normal Cell Count': 0,
                  'Color': '160; 160; 164', 'Line Style': {'Line Width': 0.029999999329447746, 'Value': 'Lines'},
                  'Enabled': True, 'Value': True, 'Plane': 'XY', 'Offset': {'Y': 0, 'X': 0, 'Z': 0}, 'Alpha': 0.5,
                  'Plane Cell Count': 10, 'Class': 'rviz/Grid'},
    'rviz/RobotModel': {'Name': 'RobotModel', 'Links': {
        'top_center_rgbd_camera_rgb_optical_frame': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False},
        'left_laser': {'Alpha': 1, 'Show Axes': False, 'Value': True, 'Show Trail': False},
        'frontal_laser_mount_link': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False},
        'right_laser': {'Alpha': 1, 'Show Axes': False, 'Value': True, 'Show Trail': False},
        'Link Tree Style': 'Links in Alphabetic Order',
        'top_right_camera': {'Alpha': 1, 'Show Axes': False, 'Value': True, 'Show Trail': False},
        'top_right_camera_optical': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False},
        'All Links Enabled': True, 'Expand Joint Details': False,
        'top_center_rgbd_camera_link': {'Alpha': 1, 'Show Axes': False, 'Value': True, 'Show Trail': False},
        'top_center_rgbd_camera_depth_optical_frame': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False},
        'frontal_laser': {'Alpha': 1, 'Show Axes': False, 'Value': True, 'Show Trail': False},
        'top_center_rgbd_camera_depth_frame': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False},
        'base_link': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False}, 'Expand Tree': False,
        'top_left_camera_optical': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False},
        'top_left_camera': {'Alpha': 1, 'Show Axes': False, 'Value': True, 'Show Trail': False},
        'Expand Link Details': False, 'model': {'Alpha': 1, 'Show Axes': False, 'Value': True, 'Show Trail': False},
        'top_center_rgbd_camera_rgb_frame': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False},
        'base_footprint': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False}},
                        'Robot Description': 'robot_description', 'Visual Enabled': True, 'Enabled': True,
                        'Value': True, 'Update Interval': 0, 'Collision Enabled': False, 'TF Prefix': '',
                        'Alpha': 1, 'Class': 'rviz/RobotModel'}
}


def execute(cmd, blocking=True, verbose=True):
    """ @brief Executes the command in the shell in a blocking or non-blocking manner
        @param cmd a string with teh command to execute
        @return
    """
    if verbose:
        print "Executing command: " + cmd
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if blocking:  # if blocking is True:
        for line in p.stdout.readlines():
            if verbose:
                print line,
            p.wait()


if __name__ == "__main__":
    # Parse command line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", "--name", help='package name', type=str, required=True)
    args = vars(ap.parse_args())

    # --------------------------------------------------------------------------
    # Check paths
    # --------------------------------------------------------------------------
    package_name = os.path.basename(args['name'])
    rospack = rospkg.RosPack()
    interactive_calibration_path = rospack.get_path('interactive_calibration')
    try:
        verified_package_path = rospack.get_path(package_name)
    except:
        s = Fore.YELLOW + 'Package ' + package_name + 'not found under ROS. Are you sure the path you gave in under ' \
                                                      'your $ROS_PACKAGE_PATH? Calibration package will not work if ' \
                                                      'not under the $ROS_PACKAGE_PATH. Please fix this. ' + \
            Style.RESET_ALL
        raise ValueError(s)

    print('Verified package path in ' + verified_package_path)

    # --------------------------------------------------------------------------
    # Read the config.yml file
    # --------------------------------------------------------------------------
    config_file = verified_package_path + '/calibration/config.yml'
    config = yaml.load(open(config_file), Loader=yaml.CLoader)

    print('\nconfig=' + str(config))

    # --------------------------------------------------------------------------
    # Setup the description file
    # --------------------------------------------------------------------------
    description_file = verified_package_path + '/urdf/description.urdf.xacro'
    # Create a symbolic link to the given xacro
    execute('ln -fs ' + config['description_file'] + ' ' + description_file)

    # --------------------------------------------------------------------------
    # Read the bag file
    # --------------------------------------------------------------------------
    bag = rosbag.Bag(config['bag_file'])
    bag_info = bag.get_type_and_topic_info()
    bag_types = bag_info[0]
    bag_topics = bag_info[1]
    # print('\n' + str(bag_topics))

    # for topic, msg, t in bag.read_messages(topics=['chatter', 'numbers']):
    #     print(msg)
    bag.close()

    # --------------------------------------------------------------------------
    # Read the description.urdf.xacro file
    # --------------------------------------------------------------------------
    # Check the description file
    urdf_file = '/tmp/description.urdf'
    execute('xacro ' + description_file + ' -o ' + urdf_file)  # create a temp urdf file
    try:
        description = URDF.from_xml_file(urdf_file)  # read teh urdf file
    except:
        raise ValueError('Could not parse description file ' + description_file)

    # print(description)
    # --------------------------------------------------------------------------
    # Verifications (run as much as we can think of to see if something is wrong)
    # --------------------------------------------------------------------------

    compressed_topics = {}  # a list of compressed topics to decompress in the launch file
    # Check if config sensor topics exist in the bag file
    for sensor_key in config['sensors']:
        topic = config['sensors'][sensor_key]['topic_name']
        if topic not in bag_topics:

            topic_compressed = topic + '/compressed'
            if topic_compressed in bag_topics:  # Check if the topic is a compressed image topic
                msg_type = bag_info[1][topic_compressed].msg_type
                if msg_type == 'sensor_msgs/CompressedImage':  # Check if the topic of correct msg_type
                    compressed_topics[topic] = {'topic_compressed': topic_compressed, 'msg_type': msg_type,
                                                'sensor_key': sensor_key}
                    print(Fore.BLUE + 'Topic ' + topic + ' is in compressed format (' + topic_compressed +
                          '). Will setup a decompressor.' + Style.RESET_ALL)
                else:
                    raise ValueError(Fore.RED + ' Topic ' + topic + ' (from sensor ' + sensor_key +
                                     ') exists in compressed format in the bag file, but is not of msg_type '
                                     '"sensor_msgs/CompressedImage"' + Style.RESET_ALL)
            else:

                raise ValueError(Fore.RED + ' Topic ' + topic + ' (from sensor ' + sensor_key +
                                 ') does not exist in the bag file.' + Style.RESET_ALL)

    # TODO Check if config links exist in the tf topics of the bag file

    # --------------------------------------------------------------------------
    # Create the launch file
    # --------------------------------------------------------------------------
    print('Setting up launch files ...')
    first_guess_launch_file = verified_package_path + '/launch/bringup_fg.launch'
    f = open(first_guess_launch_file, 'w')

    now = datetime.now()
    dt_string = now.strftime("%d/%m/%Y %H:%M:%S")

    f.write('<launch>\n' +
            '<!-- This file was generated automatically by the script configure_calibration_pkg.py on the ' + dt_string + '-->\n\n' +
            '<!-- %%%%%%%%%%%%% ATOMIC Framework %%%%%%%%%%%%%%%%%%%%%-->\n' +
            '<!-- Runs bringup first guess for initial parameter setting through rviz interactive markers-->\n' +
            '\n' +
            '<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%-->\n' +
            '<!-- Parameters-->\n' +
            '<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%-->\n' +
            '<!-- bag_file: you can give another bagfile to the launch file using this arg-->\n' +
            '<arg name="bag_file" default="' + config['bag_file'] + '"/>\n' +
            '<!-- bag_start: starting time for playing back the bag file. Check "rosbag play -h"-->\n' +
            '<arg name="bag_start" default="0"/>\n' +
            '<arg name ="bag_rate" default ="1" />\n' +
            '<arg name="description_file" default="$(find ' + package_name + ')/urdf/description.urdf.xacro"/>\n' +
            '<arg name="rviz_file" default="$(find ' + package_name + ')/rviz/config.rviz"/>\n' +
            '\n' +
            '<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%-->\n' +
            '<!-- Load robot description and tf generators -->\n' +
            '<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%-->\n' +
            '<param name = "robot_description" command = "$(find xacro)/xacro $(arg description_file)" />\n' +
            '<node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher"/>\n' +
            # '<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">\n' +
            # '    <param name="use_gui" value="true"/>\n' +
            # '</node>\n' +
            '\n' +
            '<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%-->\n' +
            '<!-- Playback the bag file -->\n' +
            '<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%-->\n' +
            '<param name="/use_sim_time" value="true"/>\n' +
            '<node pkg="rosbag" type="play" name="rosbag_play" output="screen" args=" $(arg bag_file) --clock -r $('
            'arg bag_rate) -l -s $(arg bag_start) /tf:=/tf_dev_null /tf_static:=/tf_static_dev_null"/>\n\n'
            )

    for compressed_topic in compressed_topics:  # add decompressors if needed
        print(compressed_topic)
        sensor_key = compressed_topics[compressed_topic]['sensor_key']
        f.write('<node pkg="image_transport" type="republish" name="republish_' + sensor_key +
                '" output="screen" args="compressed in:=' + compressed_topic + ' raw out:=' + compressed_topic +
                '"/>\n')

    f.write('\n<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%-->\n' +
            '<!-- Visualization -->\n' +
            '<!-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%-->\n' +
            '<node name="rviz" pkg="rviz" type="rviz" args="-d $(arg rviz_file)" required="false"/>\n'
            )

    f.write('\n\n</launch>')
    f.close()

    # --------------------------------------------------------------------------
    # Create the rviz config file
    # --------------------------------------------------------------------------
    rviz_file_template = interactive_calibration_path + '/templates/config.rviz'
    rviz = yaml.load(open(rviz_file_template), Loader=yaml.FullLoader)
    vm = rviz['Visualization Manager']
    wg = rviz['Window Geometry']
    vm_displays = vm['Displays']
    for i, vm_display in enumerate(vm_displays):
        print('Display ' + str(i) + ' is ' + vm_display['Name'] + ' Class ' + Fore.RED + vm_display[
            'Class'] + Style.RESET_ALL)
        print(vm_display)

    vm_displays = []

    # Generate rviz displays according to the sensor types
    for sensor_key in config['sensors']:
        topic = config['sensors'][sensor_key]['topic_name']
        topic_compressed = topic + '/compressed'
        if topic in bag_topics:
            msg_type = bag_info[1][topic].msg_type
        elif topic_compressed in bag_topics:
            msg_type = bag_info[1][topic_compressed].msg_type
        else:
            raise ValueError('Could not find topic ' + topic + ' in bag file.')

        print('Sensor ' + sensor_key + ' with topic ' + topic + ' (' + msg_type + ')')

        if msg_type == 'sensor_msgs/CompressedImage' or msg_type == 'sensor_msgs/Image':  # add displays for camera sensor

            # Raw data Image
            d = display_dicts['rviz/Image']
            display_name = sensor_key + '-Image'
            d['Name'] = display_name
            d['Image Topic'] = topic
            d['Enabled'] = False
            vm_displays.append(deepcopy(d))
            wg[display_name] = {'collapsed': True}
            # TODO cannot disable the marker

            # Labelled data Image
            d = display_dicts['rviz/Image']
            display_name = sensor_key + '-Labelled' + '-Image'
            d['Name'] = display_name
            d['Image Topic'] = topic + '/Labelled'
            d['Enabled'] = False
            vm_displays.append(deepcopy(d))
            wg[display_name] = {'collapsed': True}

            # Camera
            d = display_dicts['rviz/Camera']
            display_name = sensor_key + '-Camera'
            d['Name'] = display_name
            d['Image Topic'] = topic
            d['Enabled'] = False
            vm_displays.append(deepcopy(d))
            wg[display_name] = {'collapsed': True}

    vm['Displays'] = vm_displays
    vm['Window Geometry'] = wg
    rviz_file = verified_package_path + '/rviz/config.rviz'
    yaml.dump(rviz, open(rviz_file, 'w'))

    # Print final message
    print('\n\nSuccessfuly configured calibration package ' + package_name + '. You can use the launch files:\n')
    print('   ' + Fore.BLUE + 'roslaunch ' + package_name + ' ' + os.path.basename(first_guess_launch_file) +
          '\n' + Style.RESET_ALL)
