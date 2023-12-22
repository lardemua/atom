
# Standard imports
import os
import pty


import re
import subprocess

# Ros imports
from urllib.parse import urlparse

import yaml
import rospkg
from colorama import Fore, Style


def execute(cmd, blocking=True, verbose=True):
    """ @brief Executes the command in the shell in a blocking or non-blocking manner
        @param cmd a string with teh command to execute
        @return
    """
    if verbose:
        print("Executing command: " + cmd)

    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if blocking:  # if blocking is True:
        for line in p.stdout.readlines():
            if verbose:
                print(line)
            p.wait()


def execColored(cmd):
    """Using popen it is not possible to maintain colors as those used in colorama.
    This way of making system calls preserves colors.
    https://stackoverflow.com/questions/56835373/python-subprocess-realtime-print-with-colors-and-save-stdout
    Args:
        cmd (str): The command to execute
    """

    cmd_parts = cmd.split(" ")
    pty.spawn(cmd_parts, lambda fd: os.read(fd, 1024))


def resolvePath(path, verbose=False):
    """ Resolves path by replacing environment variables, common notations (e.g. ~ for home/user)"""

    path = os.path.expanduser(path)
    path = os.path.expandvars(path)
    path = os.path.abspath(path)
    path = os.path.normpath(path)
    return path


def expandToLaunchEnv(path):
    if len(path) == 0:  # if path is empty, path[0] does not exist
        return path

    if path[0] == '~':
        path = '$(env HOME)' + path[1:]

    if '$' not in path:
        return path

    evars = re.compile(r'\$(\w+|\{[^}]*\})')
    i = 0
    while True:
        m = evars.search(path, i)
        if not m:
            break

        i, j = m.span(0)
        name = m.group(1)
        if name.startswith('{') and name.endswith('}'):
            name = name[1:-1]

        tail = path[j:]
        path = path[:i] + '$(env {})'.format(name)
        i = len(path)
        path += tail

    return path
