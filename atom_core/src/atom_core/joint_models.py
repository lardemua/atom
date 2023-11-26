
import numpy as np


from transformations import euler_matrix, rotation_matrix

import numpy as np


from atom_core.utilities import atomError


def getTransformationFromRevoluteJoint(joint):

    if joint['xacro_joint_type'] is None:
        atomError('Cannot model non revolute joint.')

    # STEP 1: compute the rotation matrix due to the joint revolution position
    joint_axis = [joint['axis']['x'], joint['axis']['y'], joint['axis']['z']]
    joint_position_matrix = rotation_matrix(joint['position'], joint_axis, point=None)

    # STEP 2: compute the rotation due to the origin rpy
    origin_matrix = euler_matrix(joint['origin']['roll'],
                                 joint['origin']['pitch'],
                                 joint['origin']['yaw'], axes='sxyz')

    # STEP 3: Add to the origin matrix the translation
    origin_matrix[0, 3] = joint['origin']['x']
    origin_matrix[1, 3] = joint['origin']['y']
    origin_matrix[2, 3] = joint['origin']['z']

    # STEP 4: compute the aggregate transformation
    # TODO I get confused here, I though the multiplication should be the other way around.
    # However, this is the way it works exactly like robot state publisher.
    composed_matrix = np.dot(origin_matrix, joint_position_matrix)

    return composed_matrix
