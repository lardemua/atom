import unittest
from atom_core.atom import getChain, getAggregateTransform, getTransform

class TestGetChain(unittest.TestCase):

    def test_get_chain_straight(self):
        
        from_frame = 'frame_0'
        to_frame = 'frame_3'

        # Create an example tranform pool
        transform_pool = {} # Init the tranform_pool variable

        for i in range(0,10):
            parent_frame = 'frame_' + str(i)
            child_frame = 'frame_' + str(i+1)
            transform_pool[parent_frame + '-' + child_frame] = {'parent': parent_frame,
                                                                'child': child_frame}

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

        # Hardcode in an example transform pool with "branches"

        transform_pool = {
            'frame_0-frame_1': {
                'parent': 'frame_0',
                'child': 'frame_1'
            },
            'frame_1-frame_2': {
                'parent': 'frame_1',
                'child': 'frame_2'
            },
            # 3 branches from frame_2
            'frame_2-frame_3': {
                'parent': 'frame_2',
                'child': 'frame_3'
            },
            'frame_2-frame_4': {
                'parent': 'frame_2',
                'child': 'frame_4'
            },
            'frame_2-frame_5': {
                'parent': 'frame_2',
                'child': 'frame_5'
            },
            # 2 branches from frame_3
            'frame_3-frame_6': {
                'parent': 'frame_3',
                'child': 'frame_6'
            },
            'frame_3-frame_7': {
                'parent': 'frame_3',
                'child': 'frame_7'
            },
            # 1 branch from frame_4
            'frame_4-frame_8': {
                'parent': 'frame_4',
                'child': 'frame_8'
            },
            # 2 branches from frame_5
            'frame_5-frame_9': {
                'parent': 'frame_5',
                'child': 'frame_9'
            },
            'frame_5-frame_10': {
                'parent': 'frame_5',
                'child': 'frame_10'
            }
        }

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


# class TestGetAggregateTransform(unittest.TestCase):
#     def test_get_aggregate_transforms(self):
#         pass
        
# class TestGetTransform(unittest.TestCase):
#     def test_get_transform(self):
#         pass