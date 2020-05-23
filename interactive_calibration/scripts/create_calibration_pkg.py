#!/usr/bin/env python

# ------------------------
#    IMPORT MODULES      #
# ------------------------
import argparse
import os
import rospkg
import subprocess
from colorama import Style, Fore
from datetime import datetime

import rospy


def execute(cmd, blocking=True, verbose=False):
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
    ap.add_argument("-n", "--name", help='package_name', type=str, required=True)
    args = vars(ap.parse_args())

    # Create package
    package_name = os.path.basename(args['name'])
    package_path = os.path.dirname(args['name'])
    if not package_name:
        execute('cd ' + package_path, verbose=True)
    else:
        print('Creating package on current directory.')
        package_path = os.getcwd()

    print('Creating package ' + package_name + ' on path ' + package_path)
    cmd = 'cd ' + package_path + ' && catkin_create_pkg ' + package_name + ' rospy' + \
          '-a "Miguel Riem Oliveira" ' + '-m "Miguel Riem Oliveira" ' + \
          '-D "This package was created automatically by the create_calibration_pkg.py ' \
          'script. Check the ATOMIC framework webpage for more information."'
    execute(cmd, verbose=True)

    execute('rospack profile && rospack profile && rospack profile')

    # Check if package is under $ROS_PACKAGE_PATH
    rospack = rospkg.RosPack()
    interactive_calibration_path = rospack.get_path('interactive_calibration')
    try:
        verified_package_path = rospack.get_path(package_name)
    except:
        print(Fore.YELLOW + 'Package ' + package_name +
              ' not found under ROS. Are you sure the path you gave in under your $ROS_PACKAGE_PATH? Calibration ' \
              'package will not work if not under the $ROS_PACKAGE_PATH. Please fix this. ' + Style.RESET_ALL)
        verified_package_path = package_path + '/' + package_name

    # Create package structure (e.g. create folders, etc)
    execute('mkdir ' + verified_package_path + '/calibration')
    execute('mkdir ' + verified_package_path + '/launch')
    execute('mkdir ' + verified_package_path + '/rviz')
    execute('mkdir ' + verified_package_path + '/urdf')
    execute('mkdir ' + verified_package_path + '/scripts')

    # Add configure.py script
    configure_file = verified_package_path + '/scripts/configure'
    f = open(configure_file,'w')
    f.write('#!/bin/sh') # shebang

    now = datetime.now()
    dt_string = now.strftime("%d/%m/%Y %H:%M:%S")
    f.write('\n# File created automatically on ' + dt_string)
    f.write('\n#      ATOMIC Framework')
    f.write('\nrosrun interactive_calibration configure_calibration_pkg.py -n ' + package_name)
    f.close()
    execute('chmod +x ' + configure_file) # make file executable

    # Add template config.json
    config_file_src = interactive_calibration_path + '/templates/config.yml'
    config_file_dst = verified_package_path + '/calibration/config.yml'
    cmd = 'cp ' + config_file_src + ' ' + config_file_dst
    execute(cmd)

    # Print final message
    print('\n\nCreated calibration package ' + package_name + ' in ' + verified_package_path + '. Now you should:')
    print(Fore.BLUE + '   1. Edit the ' + config_file_dst + ' to setup the calibration and; \n   2. Run ' + Fore.RED +
          ' calibration_setup ' + Style.RESET_ALL)
