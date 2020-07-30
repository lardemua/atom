#!/usr/bin/env python
"""
Reads a set of data and labels from a group of sensors in a json file and calibrates the poses of these sensors.
"""

import inspect


# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------

class Cache:
    """
    A caching mechanism to be used as function decorator. The arguments of the function determine the key to the
    cached map, and the values are the return of the function call.
    The class works by creating a cache dict that contains:
        as keys a description of the function arguments names
        as values the returned value of the function when supplied with those values for the parameters
    The key is created automatically but to do so, all arguments must be hashable. If one or more arguments are not
    hashable it is possible to ignore them by adding their name to the kwargs_to_ignore list of strings. If you do this,
    be sure to check that that ignored argument does not affect the function output, otherwise a cached return could be
    used when the function call would give a different result.

    To profile mark the function(s) you want to profile with decorator @profile, then run the script with
        >> kernprof -l -v your_script.py scripts_arg1, ... script argn

    An example:
        >> kernprof -l -v scripts/calibrate -json /home/mike/datasets/agrob/agrob_01_07_2020/data_collected.json
     -csf 'lambda x: int(x) < 1' -ssf 'lambda name: name in ["left_camera","right_camera", "vlp16"]' -oi -rv -si -sr 0.3
    """

    def __init__(self, args_to_ignore=[], verbose=False, disable_cache=False):
        self.args_to_ignore = args_to_ignore
        self.cache = dict()  # initialize cache
        self.verbose = verbose
        self.disable_cache = disable_cache

    def __call__(self, func):
        def wrapper(*args, **kwargs):

            # Generate key from the hashable args which are not in kwargs_to_ignore
            call_args = inspect.getcallargs(func, *args, **kwargs)
            for key_to_ignore in self.args_to_ignore:
                call_args.pop(key_to_ignore, None)  # remove args to ignore

            if self.verbose:
                print('\nFunction call with call_args (ignored args ommited):' + str(call_args))

            key = '_'.join('{}={}'.format(k, v) for k, v in call_args.items())  # create key

            if self.verbose:
                print('key = ' + str(key))

            if self.cache.has_key(key) and not self.disable_cache:  # if key exists then use it
                r = self.cache[key]
                if self.verbose:
                    print('Using stored value')
            else:  # if key does not exist run function and store return value
                r = func(*args, **kwargs)
                self.cache[key] = r
                if self.verbose:
                    print('No stored value, calling function')

            if self.verbose:
                print('Current cache keys:\n' + str(self.cache.keys()))
            return r

        return wrapper

    def clearCache(self):
        self.cache = dict()
