import unittest
import json

import numpy as np
from atom_core.atom import getChain, getAggregateTransform, getTransform

class TestGetChain(unittest.TestCase):

    def test_get_chain_straight(self):
        
        from_frame = 'frame_0'
        to_frame = 'frame_3'

        # Open testing transform pool
        with open('test_inputs/atom_000.json') as f:
            transform_pool = json.load(f)

        # Call the function
        func_result = getChain(from_frame, to_frame, transform_pool)

        # Hardcode the correct result
        correct_result = [
            {'parent': 'frame_0',
             'child': 'frame_1',
             'key': 'frame_0-frame_1'},
            {'parent': 'frame_1',
             'child': 'frame_2',
             'key': 'frame_1-frame_2'},
            {'parent': 'frame_2',
            'child': 'frame_3',
            'key': 'frame_2-frame_3'}
        ]

        self.assertEqual(correct_result, func_result)

    def test_get_chain_branching(self):

        from_frame = 'frame_0'
        to_frame = 'frame_7'

        # Open testing transform pool
        with open('test_inputs/atom_001.json') as f:
            transform_pool = json.load(f)
        
        # Call the function
        func_result = getChain(from_frame, to_frame, transform_pool)

        # Hardcode the correct result
        correct_result = [
            {'parent': 'frame_0',
             'child': 'frame_1',
             'key': 'frame_0-frame_1'},
            {'parent': 'frame_1',
             'child': 'frame_2',
             'key': 'frame_1-frame_2'},
            {'parent': 'frame_2',
            'child': 'frame_3',
            'key': 'frame_2-frame_3'},
            {'parent': 'frame_3',
            'child': 'frame_7',
            'key': 'frame_3-frame_7'}
        ]

        self.assertEqual(correct_result, func_result)


class TestGetAggregateTransform(unittest.TestCase):
    def test_get_aggregate_transforms_eye_x_eye(self):
        # A simple sanity check
        
        with open('test_inputs/atom_002.json') as f:
            transform_pool = json.load(f)

        chain = [
            {'parent': 'frame_0',
             'child': 'frame_1',
             'key': 'frame_0-frame_1'},
            {'parent': 'frame_1',
             'child': 'frame_2',
             'key': 'frame_1-frame_2'},
            {'parent': 'frame_2',
            'child': 'frame_3',
            'key': 'frame_2-frame_3'}
        ]

        correct_result = np.eye(4, dtype=float)

        func_result = getAggregateTransform(chain, transform_pool)

        np.testing.assert_array_equal(correct_result, func_result)

        
# class TestGetTransform(unittest.TestCase):
#     def test_get_transform(self):
#         pass