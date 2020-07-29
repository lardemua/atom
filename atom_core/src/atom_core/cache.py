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
    A caching mechanism to be used as function decorators. The arguments of the function determine the key to the
    cached map, and the values are the return of the function call.
    """
    def __init__(self, kwargs_to_ignore=[]):
        self.kwargs_to_ignore = kwargs_to_ignore
        self.cache = dict()

    def __call__(self, func):
        def wrapper(*args, **kwargs):

            # Generate key from the hashable args which are not in kwargs_to_ignore
            call_args = inspect.getcallargs(func, *args, **kwargs)
            for key_to_ignore in self.kwargs_to_ignore:
                call_args.pop(key_to_ignore, None) # remove args to ignore
            key = '_'.join('{}={}'.format(k, v) for k, v in call_args.items()) # create key
            # print('key = ' + str(key))

            if self.cache.has_key(key): # if key exists then use it
                r = self.cache[key]
                # print('Using stored value')
            else: # if key does not exist run function and store return value
                r = func(*args, **kwargs)
                self.cache[key] = r
                # print('Calling function')

            # print(self.cache)
            return r

        return wrapper



