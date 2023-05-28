#!/usr/bin/env python
"""
Reads a set of data and labels from a group of sensors in a json file and calibrates the poses of these sensors.
"""

# stdlib

# 3rd-party
import math
import os
import subprocess
from statistics import mean
from signal import setitimer, signal, SIGALRM, ITIMER_REAL

import readchar
import rospkg
from colorama import Fore, Style
from atom_core.config_io import execute

# 3rd-party
from rospy_message_converter import message_converter
from pynput import keyboard

# from open3d import * # This cannot be used. It itereferes with the Image for getMessageTypeFromTopic(topic):


# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------

def raise_timeout_error(signum, frame):
    raise subprocess.TimeoutExpired(None, 1)


def input_with_timeout(prompt, timeout):
    # set signal handler
    signal(SIGALRM, raise_timeout_error)
    setitimer(ITIMER_REAL, timeout)

    try:
        k = readchar.readkey()
        print('received key')
        return k
    except:
        # print('Timeout')
        pass
    finally:
        # print('finally')
        # signal.alarm(0)  # cancel alarm
        setitimer(ITIMER_REAL, 0)


def waitForKeyPress2(function=None, timeout=5, message='Waiting ... '):
    message = message + '\npress ' + Fore.BLUE + Style.BRIGHT + '"c"' + Style.RESET_ALL + \
        ' to continue or ' + Fore.BLUE + Style.BRIGHT + '"q"' + Style.RESET_ALL + ' to abort.'

    while True:
        if not function is None:
            print('Calling function')
            function()

        print(message)
        key = input_with_timeout('', 1)
        print('key: ' + str(key))
        if key == 'c':
            print('\nyou pressed c ... continuing.')
            break
        elif key == 'q':
            print('\nyou pressed q ... aborting.')
            exit(0)


def waitForKeyPress(function=None, timeout=5, message='Waiting ... '):
    message = message + '\npress ' + Fore.BLUE + Style.BRIGHT + '"c"' + Style.RESET_ALL + \
        ' to continue or ' + Fore.BLUE + Style.BRIGHT + '"q"' + Style.RESET_ALL + ' to abort.'

    while True:
        with keyboard.Events() as events:
            if not function is None:
                print('Calling function')
                function()

            print(message)
            event = events.get(timeout)
            if not event is None:
                if hasattr(event.key, 'char'):
                    if event.key.char == 'c':
                        print('\nyou pressed c ... continuing.')
                        break
                    elif event.key.char == 'q':
                        print('\nyou pressed q ... aborting.')
                        exit(0)


# Check https://stackoverflow.com/questions/52431265/how-to-use-a-lambda-as-parameter-in-python-argparse
def create_lambda_with_globals(s):
    return eval(s, globals())


def laser_scan_msg_to_xy(msg):
    data = message_converter.convert_ros_message_to_dictionary(msg)
    return laser_scan_data_to_xy(data)


def laser_scan_data_to_xy(data):
    ranges = data['ranges']
    angle_min = data['angle_min']
    angle_increment = data['angle_increment']

    x = []
    y = []
    for range_idx, r in enumerate(ranges):
        theta = angle_min + angle_increment * range_idx
        x.append(r * math.cos(theta))
        y.append(r * math.sin(theta))

    return x, y


def checkDirectoryExistence(directory, package_name, create_if_nonexistent=False):
    package_path = rospkg.RosPack().get_path(package_name)  # full path to the package, including its name.

    if not os.path.exists(package_path + '/' + directory):
        if create_if_nonexistent:
            print(Fore.YELLOW + directory + Style.RESET_ALL + ' directory does not exist. Created a new one.')
            execute('mkdir ' + package_path + '/' + directory)
            return True
        else:
            print(Fore.RED + directory + Style.RESET_ALL + ' directory does not exist.')
            return False


def rootMeanSquare(errors):
    """Computes mean square error of a list of error values.

    Args:
        errors (list of float): the list of errors
    """

    return math.sqrt(mean([e**2 for e in errors]))

def checkAdditionalTfs(dataset):
    if 'additional_tfs' in dataset['calibration_config']:
        if dataset['calibration_config']['additional_tfs'] != "":
            return True
        else:
            return False


def atomError(message):
    print(Fore.RED + 'ATOM Error: ' + Style.RESET_ALL + message)
    exit(0)