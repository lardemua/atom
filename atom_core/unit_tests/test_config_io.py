import unittest
from unittest.mock import Mock, patch
from copy import deepcopy
from atom_core.config_io import dictionaries_have_same_keys, verifyConfig
from atom_core.utilities import atomError


class TestDictionariesHaveSameKeys(unittest.TestCase):

    def test_dictionaries_have_same_keys_all_equal(self):
        d1 = {
            "key1": "value1",
            "key2": "value2",
            "key3": "value3"
            }
        
        d2 = deepcopy(d1)

        correct_result = (True, [], [])

        func_result = dictionaries_have_same_keys(d1, d2)

        self.assertEqual(correct_result, func_result)


    def test_dictionaries_have_same_keys_with_extra_key(self):
    
        d1 = {
            "key1": "value1",
            "key2": "value2",
            "key3": "value3"
            }
        
        d2 = {
            "key2": "value2",
            "key3": "value3"
            }

        correct_result = (False, ["key1"], [])

        func_result = dictionaries_have_same_keys(d1, d2)

        self.assertEqual(correct_result, func_result)

    def test_dictionaries_have_same_keys_with_missing_key(self):
    
        d1 = {
            "key1": "value1",
            "key2": "value2",
            "key3": "value3"
            }
        
        d2 = {
            "key1": "value1",
            "key2": "value2",
            "key3": "value3",
            "key4": "value4"
            }

        correct_result = (False, [], ["key4"])

        func_result = dictionaries_have_same_keys(d1, d2)

        self.assertEqual(correct_result, func_result)
    
    def test_dictionaries_have_same_keys_with_extra_and_missing_key(self):
    
        d1 = {
            "key1": "value1",
            "key2": "value2",
            "key3": "value3"
            }
        
        d2 = {
            "key2": "value2",
            "key3": "value3",
            "key4": "value4"
            }

        correct_result = (False, ["key1"], ["key4"])

        func_result = dictionaries_have_same_keys(d1, d2)

        self.assertEqual(correct_result, func_result)



