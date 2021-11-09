#!/usr/bin/env python
"""
Reads a set of data and labels from a group of sensors in a json file and calibrates the poses of these sensors.
"""

# stdlib

# 3rd-party
import math
from colorama import Fore, Style

# 3rd-party
from rospy_message_converter import message_converter
from pynput import keyboard


# from open3d import * # This cannot be used. It itereferes with the Image for getMessageTypeFromTopic(topic):



# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------

# def alarm_handler(signum, frame):
#     raise TimeoutExpired
#
# def input_with_timeout(prompt, timeout):
#     # set signal handler
#     signal.signal(signal.SIGALRM, alarm_handler)
#     signal.alarm(timeout)  # produce SIGALRM in `timeout` seconds
#
#     try:
#         return readchar()
#     finally:
#         signal.alarm(0)  # cancel alarm
#
# def waitForKeyPress2(function=None, timeout=5, message='Waiting ... ' ):
#
#     message = message + '\npress ' + Fore.BLUE + Style.BRIGHT + '"c"' + Style.RESET_ALL + ' to continue or ' + Fore.BLUE + Style.BRIGHT + '"q"' + Style.RESET_ALL + ' to abort.'
#
#     # while True:
#     #     with keyboard.Events() as events:
#     #         if not function is None:
#     #             print('Calling function')
#     #             function()
#     #
#     #         print(message)
#     #         event = events.get(timeout)
#     #         if not event is None:
#     #             if hasattr(event.key, 'char'):
#     #                 if event.key.char == 'c':
#     #                     print('\nyou pressed c ... continuing.')
#     #                     break
#     #                 elif event.key.char == 'q':
#     #                     print('\nyou pressed q ... aborting.')
#     #                     exit(0)


def waitForKeyPress(function=None, timeout=5, message='Waiting ... ' ):

    message = message + '\npress ' + Fore.BLUE + Style.BRIGHT + '"c"' + Style.RESET_ALL + ' to continue or ' + Fore.BLUE + Style.BRIGHT + '"q"' + Style.RESET_ALL + ' to abort.'

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


