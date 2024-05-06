import cv2
import numpy as np

eye_coords = np.array([[0.0, 0.0, 0.0], [0.0, 1.0, 0.0],
                      [1.0, 1.0, 0.0], [1.0, 0.0, 0.0]])
hand_coords = np.array([[0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [
                       1.0, 1.0, 0.0], [1.0, 0.0, 0.0]])
# It is the rotation matrix between the hand and eye
R_target2cam = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [
                        0.0, 0.0, 1.0], [0.0, 0.0, 0.0]])
# It is the translation between hand and eye

x_target2cam = np.array([0.0, 0.0, 0.0, 0.0])
# It is code for 3*4 matrix transformation
X, _ = cv2.calibrateHandEye(hand_coords, eye_coords,
                            R_target2cam, t_target2cam)
print(X)
