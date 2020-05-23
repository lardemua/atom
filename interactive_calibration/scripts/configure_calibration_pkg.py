#!/usr/bin/env python

# ------------------------
#    IMPORT MODULES      #
# ------------------------
import argparse
import os
import rospkg
import subprocess
from datetime import date, datetime

import yaml
from colorama import Style, Fore

import rosbag
import rospy

from urdf_parser_py.urdf import URDF


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
    # ap.add_argument("-b", "--bag_file", help='bag file name', type=str, required=True)
    # ap.add_argument("-d", "--description_file", help='file name of the xacro description file', type=str, required=True)
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

    # Print final message
    print('\n\nSuccessfuly configured calibration package ' + package_name + '. You can use the launch files:\n')
    print('   ' + Fore.BLUE + 'roslaunch ' + package_name + ' ' + os.path.basename(first_guess_launch_file) +
          '\n' + Style.RESET_ALL)
