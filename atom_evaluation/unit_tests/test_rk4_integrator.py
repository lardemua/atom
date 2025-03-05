import json
import unittest
import numpy as np

from atom.atom_evaluation.scripts.other_calibrations.imu_calibrations.imu_integrator import normalize_quaternion, omega, rk4_imu_integration, skew_sym_matrix

class TestSkewSymMatrix(unittest.TestCase):
    
    def test_skew_symmetric_matrix_simple_case(self):
        
        input_vector = [3, -1, 5] # Example of a random vector

        correct_res = np.array([[0, -5, -1],
                                [5,  0, -3],
                                [1,  3,  0]])
        
        function_res = skew_sym_matrix(input_vector)

        np.testing.assert_array_equal(correct_res, function_res)

class TestOmega(unittest.TestCase):
    def test_omega_simple_case(self):

        input_vector = np.array([3, -1, 5]) # Example of a random vector

        correct_res = np.array([[0,  -3,  1, -5],
                                [3,   0,  5,  1],
                                [-1, -5,  0,  3],
                                [5,  -1, -3,  0]])
        
        function_res = omega(input_vector)

        np.testing.assert_array_equal(correct_res, function_res)

class TestNormalizeQuaternion(unittest.TestCase):
    def test_normalize_quaternion_simple_case(self):
        input_quaternion = [12, 5436, -54565, 40]

        normalized_quaternion = normalize_quaternion(input_quaternion)

        function_vector_norm = np.linalg.norm(normalized_quaternion)

        correct_vector_norm = 1

        self.assertAlmostEqual(correct_vector_norm, function_vector_norm)

class TestRK4Integrator(unittest.TestCase):
    def test_rk4_integrator_null_vel(self):
        # Simple test with null values for velocities
        # The result of the integration should be an angular displacement of 0 on all axes
        with open('test_data/rk4_integrator_000.json') as f:
            input_data = json.load(f)

        input_data_0 = input_data[0]
        input_data_1 = input_data[1]

        correct_res = np.array([1.0, 0.0, 0.0, 0.0])

        func_res = rk4_imu_integration(input_data_0, input_data_1)

        np.testing.assert_almost_equal(correct_res, func_res)

    def test_rk4_integrator_const_vel(self):
        # Simple test with constant velocities
        # With a constant angular velocity of 1 rad/s, the integrator should return an estimated dtheta of 1 rad along x

        with open('test_data/rk4_integrator_001.json') as f:
            input_data = json.load(f)

        input_data_0 = input_data[0]
        input_data_1 = input_data[1]

        correct_res = np.array([0.8775826, 0.4794255, 0.0, 0.0])

        func_res = rk4_imu_integration(input_data_0, input_data_1)

        np.testing.assert_allclose(correct_res, func_res, rtol=1e-3)
