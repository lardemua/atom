
# Standard imports
import os
import re
import subprocess

# Ros imports
from urllib.parse import urlparse

import yaml
import rospkg
from colorama import Fore


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
                print
                line,
            p.wait()


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


def uriReader(resource):
    uri = urlparse(str(resource))
    # print(uri)
    if uri.scheme == 'package':  # using a ros package uri
        # print('This is a ros package')
        rospack = rospkg.RosPack()
        assert (rospack.get_path(uri.netloc)), 'Package ' + uri.netloc + ' does not exist.'
        fullpath = resolvePath(rospack.get_path(uri.netloc) + uri.path)
        relpath = '$(find {}){}'.format(uri.netloc, uri.path)

    elif uri.scheme == 'file':  # local file
        # print('This is a local file')
        fullpath = resolvePath(uri.netloc + uri.path)
        relpath = fullpath
    elif uri.scheme == '':  # no scheme, assume local file
        # print('This is a local file')

        fullpath = resolvePath(uri.path)
        relpath = expandToLaunchEnv(uri.path)
    else:
        raise ValueError('Cannot parse resource "' + resource + '", unknown scheme "' + uri.scheme + '".')

    assert (os.path.exists(fullpath)), Fore.RED + fullpath + ' does not exist.'
    return fullpath, os.path.basename(fullpath), relpath


def verifyConfig(config, template_config, upper_key=None):
    missing_keys = []
    # print(config)
    for key in template_config:
        # print('Checking key ' + key)
        if not key in config:
            if upper_key is None:
                missing_keys.append(key)
            else:
                missing_keys.append(upper_key + '/' + key)
            # print(str(key) + ' is not here: ' + str(config))
        elif type(config[key]) is dict and not key == 'sensors':
            # print('key ' + key + ' is a dict')

            if upper_key is None:
                mk = verifyConfig(config[key], template_config[key], key)
            else:
                mk = verifyConfig(config[key], template_config[key], upper_key + '/' + key)
            missing_keys.extend(mk)

    return missing_keys


def loadConfig(filename, check_paths=True):
    config = loadYMLConfig(filename)

    # if "robot_name" not in config.keys():  # in config:
    #     raise ValueError(Fore.RED +
    #         'Error: argument robot_name is missing in config.yaml'+ Style.RESET_ALL)
    # exit(0)
    # Check if config has all the necessary keys.
    rospack = rospkg.RosPack()
    template_file = rospack.get_path('atom_calibration') + '/templates/config.yml'
    template_config = loadYMLConfig(template_file)
    missing_parameters = verifyConfig(config, template_config)
    print(missing_parameters)

    if missing_parameters:  # list is not empty
        raise ValueError(Fore.RED + 'Your config file ' + filename +
                         ' appears to be corrupted. These mandatory parameters are missing: ' + Fore.BLUE +
                         str(missing_parameters) + Fore.RED + '\nPerhaps you should re-run:\n' + Fore.BLUE +
                         ' rosrun <your_robot>_calibration configure' + Fore.RESET)

    # Check if description file is ok
    fullpath, name, uri = uriReader(config['description_file'])

    # Check if bag_file is ok
    fullpath, name, uri = uriReader(config['bag_file'])

    # Check if calibration_pattern/mesh_file is ok
    fullpath, name, uri = uriReader(config['calibration_pattern']['mesh_file'])

    return config


def loadYMLConfig(filename):
    """Load configuration from a yml file"""
    try:
        with open(filename, 'r') as f:
            obj = yaml.load(f, Loader=yaml.SafeLoader)
    except OSError as e:
        print("I/O error({0}): {1}".format(e.errno, e.strerror))
        return None

    return obj


def validateLinks(world_link, sensors, urdf):
    try:
        for name, sensor in sensors.items():
            chain = urdf.get_chain(world_link, sensor.link)
            if sensor.parent_link not in chain or sensor.child_link not in chain:
                print("{}: The links '{}' and '{}' are not part of the same chain.".format(sensor.name,
                                                                                           sensor.parent_link,
                                                                                           sensor.child_link))
                return False
    except KeyError as e:
        link_name = str(e).strip("'")
        if link_name == urdf.get_root():
            print("Configuration contains an unknown base link: {}".format(world_link))
            return False

        print("Configuration contains an unknown link: {}".format(link_name))
        return False

    return True
