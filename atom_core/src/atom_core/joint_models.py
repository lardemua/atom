
import numpy as np


from transformations import euler_matrix, rotation_matrix, quaternion_from_matrix, translation_from_matrix
import numpy as np
from atom_core.utilities import atomError


def getTransformationFromRevoluteJoint(joint):

    if joint['joint_type'] is None:
        atomError('Cannot model non revolute joint.')

    # STEP 1: compute the rotation matrix due to the joint revolution position
    joint_axis = [joint['axis_x'], joint['axis_y'], joint['axis_z']]
    joint_position_matrix = rotation_matrix(joint['position']+joint['position_bias'], joint_axis, point=None)

    # STEP 2: compute the rotation due to the origin rpy
    origin_matrix = euler_matrix(joint['origin_roll'],
                                 joint['origin_pitch'],
                                 joint['origin_yaw'], axes='sxyz')

    # STEP 3: Add to the origin matrix the translation
    origin_matrix[0, 3] = joint['origin_x']
    origin_matrix[1, 3] = joint['origin_y']
    origin_matrix[2, 3] = joint['origin_z']

    # STEP 4: compute the aggregate transformation
    # TODO I get confused here, I though the multiplication should be the other way around.
    # However, this is the way it works exactly like robot state publisher.
    composed_matrix = np.dot(origin_matrix, joint_position_matrix)

    quat = quaternion_from_matrix(composed_matrix)
    # TODO  for some strange reason, quaternion_from_matrix returns w,x,y,z instead of x,y,z,w
    quat = [quat[1], quat[2], quat[3], quat[0]]

    trans = list(translation_from_matrix(composed_matrix))

    return quat, trans
