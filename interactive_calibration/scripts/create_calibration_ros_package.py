#!/usr/bin/env python

# ------------------------
#    IMPORT MODULES      #
# ------------------------
import argparse
import os
import rospkg
import subprocess
from colorama import Style, Fore

import rospy


##
# @brief Executes the command in the shell in a blocking or non-blocking manner
#
# @param cmd a string with teh command to execute
#
# @return
def execute(cmd, blocking=True, silent=False):
    print "Executing command: " + cmd
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if blocking:  # if blocking is True:
        for line in p.stdout.readlines():
            if not silent:
                print line,
            p.wait()


if __name__ == "__main__":
    # Parse command line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", "--name", help='package_name', type=str, required=True)
    args = vars(ap.parse_args())

    # Create package
    package_name = os.path.basename(args['name'])
    package_path = os.path.dirname(args['name'])
    if not package_name == '':
        execute('cd ' + package_path)

    print('Creating package ' + package_name + ' on path ' + package_path)
    cmd = 'cd ' + package_path + ' && catkin_create_pkg ' + package_name + ' rospy' + \
          '-a "Miguel Riem Oliveira" ' + '-m "Miguel Riem Oliveira" ' + \
          '-D "This package was created automatically by the create_calibration_ros_package.py ' \
          'script. Check the ATOMIC framework webpage for more information."'
    execute(cmd)

    execute('rospack profile && rospack profile && rospack profile', silent=True)

    # Check package under $ROS_PACKAGE_PATH
    rospack = rospkg.RosPack()
    interactive_calibration_path = rospack.get_path('interactive_calibration')
    try:
        verified_package_path = rospack.get_path(package_name)
    except:
        s = 'Package ' + package_name + 'not found. Are you sure the path you gave in under your $ROS_PACKAGE_PATH? ' \
                                        'Calibration package creation failed. Please delete the folder. '
        raise ValueError(s)

    # Create package structure (e.g. create folders, etc)
    execute('mkdir ' + verified_package_path + '/calibration')  # create calibration directory
    execute('mkdir ' + verified_package_path + '/launch')  # create launch directory
    execute('mkdir ' + verified_package_path + '/rviz')  # create launch directory

    # Add template config.json
    config_file_src = interactive_calibration_path + '/templates/config.json'
    config_file_dst = verified_package_path + '/calibration/config.json'
    cmd = 'cp ' + config_file_src + ' ' + config_file_dst
    execute(cmd)

    # Print final message
    print('\n\nCreated calibration package ' + package_name + ' in ' + verified_package_path + '. Now you should:')
    print(Fore.BLUE + '   1. Edit the ' + config_file_dst + ' to setup the calibration and; \n   2. Run ' + Fore.RED +
          ' calibration_setup ' + Style.RESET_ALL)
