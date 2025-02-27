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

    def test_get_aggregate_transform_simple(self):
        # Test for a simple tf aggregation
        
        with open('test_inputs/atom_003.json') as f:
            transform_pool = json.load(f)

        chain = [
            {'parent': 'frame_0',
             'child': 'frame_1',
             'key': 'frame_0-frame_1'},
            {'parent': 'frame_1',
             'child': 'frame_2',
             'key': 'frame_1-frame_2'}
        ]

        correct_result = np.array([[-2.36906390e-04,  1.48658100e-03,  9.99998867e-01, -1.34150242e-03],
                                   [-9.99994535e-01, -3.29799526e-03, -2.32002621e-04,  1.05864562e-01],
                                   [ 3.29764663e-03, -9.99993457e-01,  1.48735419e-03,  9.19434130e-01],
                                   [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

        func_result = getAggregateTransform(chain, transform_pool)

        np.testing.assert_array_almost_equal(correct_result, func_result)

    def test_get_aggregate_transform_reverse(self):
        # Test for a simple tf aggregation when the chain has tfs which are inverted w.r.t. the dataset/tf pool
        
        with open('test_inputs/atom_003.json') as f:
            transform_pool = json.load(f)

        chain = [
            {'parent': 'frame_2',
             'child': 'frame_1',
             'key': 'frame_2-frame_1'},
            {'parent': 'frame_1',
             'child': 'frame_0',
             'key': 'frame_1-frame_0'}
        ]

        correct_result = np.array([[-2.36906390e-04, -9.99994534e-01,  3.29764663e-03,  1.02831697e-01],
                                   [ 1.48658100e-03, -3.29799526e-03, -9.99993456e-01,  9.19779249e-01],
                                   [ 9.99998867e-01, -2.32002621e-04,  1.48735419e-03, -1.46245068e-06],
                                   [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00],]
                                  )
                                  
        func_result = getAggregateTransform(chain, transform_pool)

        np.testing.assert_array_almost_equal(correct_result, func_result)

    def test_get_aggregate_transform_invalid_chain(self):
        
        with open('test_inputs/atom_003.json') as f:
            transform_pool = json.load(f)
        
        chain = [
            {'parent': 'frame_0',
             'child': 'frame_2',
             'key': 'frame_0-frame_2'},
        ]
        
        self.assertRaises(ValueError, getAggregateTransform, chain, transform_pool)

class TestGetTransform(unittest.TestCase):
    
    def test_get_transform(self):
        
        # Since getTransform merely calls getChain and getAggregateTransform, and these have been tested, we only need to test this once

        with open('test_inputs/atom_003.json') as f:
            transform_pool = json.load(f)

        from_frame = 'frame_0'
        to_frame = 'frame_2'

        correct_result = np.array([[-2.36906390e-04,  1.48658100e-03,  9.99998867e-01, -1.34150242e-03],
                                   [-9.99994535e-01, -3.29799526e-03, -2.32002621e-04,  1.05864562e-01],
                                   [ 3.29764663e-03, -9.99993457e-01,  1.48735419e-03,  9.19434130e-01],
                                   [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
        
        func_result = getTransform(from_frame, to_frame, transform_pool)

        np.testing.assert_array_almost_equal(correct_result, func_result)