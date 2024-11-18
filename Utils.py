import matplotlib
from scipy.spatial.transform import Rotation as R

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.proj3d import proj_transform
from mpl_toolkits.mplot3d.axes3d import Axes3D
import numpy as np
from matplotlib.patches import FancyArrowPatch


class Arrow3D(FancyArrowPatch):

    def __init__(self, x, y, z, dx, dy, dz, *args, **kwargs):
        super().__init__((0, 0), (0, 0), *args, **kwargs)
        self._xyz = (x, y, z)
        self._dxdydz = (dx, dy, dz)

    def draw(self, renderer):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = (x1 + dx, y1 + dy, z1 + dz)

        xs, ys, zs = proj_transform((x1, x2), (y1, y2), (z1, z2), self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        super().draw(renderer)

    def do_3d_projection(self, renderer=None):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = (x1 + dx, y1 + dy, z1 + dz)

        xs, ys, zs = proj_transform((x1, x2), (y1, y2), (z1, z2), self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))

        return np.min(zs)


def _arrow3D(ax, x, y, z, dx, dy, dz, *args, **kwargs):
    '''Add an 3d arrow to an `Axes3D` instance.'''

    arrow = Arrow3D(x, y, z, dx, dy, dz, *args, **kwargs)
    ax.add_artist(arrow)


setattr(Axes3D, 'arrow3D', _arrow3D)

def axis_angle_to_rotation_matrix(axis, angle):
    """
    Convert an axis-angle representation to a rotation matrix.

    Parameters:
    axis (array-like): The axis of rotation (a 3-element array or list).
    angle (float): The angle of rotation in radians.

    Returns:
    np.ndarray: A 3x3 rotation matrix.
    """
    axis = np.asarray(axis)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    cos_angle = np.cos(angle)
    sin_angle = np.sin(angle)
    one_minus_cos = 1 - cos_angle
    rotation_matrix = np.array([
        [cos_angle + x ** 2 * one_minus_cos, x * y * one_minus_cos - z * sin_angle,
         x * z * one_minus_cos + y * sin_angle],
        [y * x * one_minus_cos + z * sin_angle, cos_angle + y ** 2 * one_minus_cos,
         y * z * one_minus_cos - x * sin_angle],
        [z * x * one_minus_cos - y * sin_angle, z * y * one_minus_cos + x * sin_angle,
         cos_angle + z ** 2 * one_minus_cos]
    ])
    return rotation_matrix


def create_transformation_matrix_from_axisangle(axis, angle, translation):
    """
    Create a 4x4 transformation matrix from an axis-angle representation and translation.

    Parameters:
    axis (array-like): The axis of rotation (a 3-element array or list).
    angle (float): The angle of rotation in radians.
    translation (array-like): The translations along x, y, and z axes (a 3-element array or list).

    Returns:
    np.ndarray: A 4x4 transformation matrix.
    """
    # Convert axis-angle to rotation matrix
    rotation_matrix = axis_angle_to_rotation_matrix(axis, angle)

    # Initialize a 4x4 identity matrix
    transformation_matrix = np.eye(4)

    # Insert the rotation matrix into the top-left 3x3 part of the transformation matrix
    transformation_matrix[:3, :3] = rotation_matrix

    # Insert the translation values into the last column of the transformation matrix
    transformation_matrix[0, 3] = translation[0]
    transformation_matrix[1, 3] = translation[1]
    transformation_matrix[2, 3] = translation[2]

    return transformation_matrix

def axis_from_rotation_matrix(matrix):
    rotation = R.from_matrix(matrix[:3,:3])
    axis_angle = rotation.as_rotvec()
    angle = np.linalg.norm(axis_angle)
    axis = axis_angle / angle if angle != 0 else axis_angle
    return axis

def angle_from_rotation_matrix(matrix):
    rotation = R.from_matrix(matrix[:3,:3])
    angle = np.linalg.norm(rotation.as_rotvec())
    return angle


def quaternion_to_matrix(quaternion):
    x, y, z, w = quaternion
    return np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w, 0],
        [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w, 0],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2, 0],
        [0, 0, 0, 1]
    ])

def create_transformation_matrix(quaternion, translation):
    rotation_matrix = quaternion_to_matrix(quaternion)
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix[:3, :3]
    transformation_matrix[:3, 3] = translation
    return transformation_matrix


def quaternion_rotation_angle_between(q1, q2):
    """
    Calculate the rotation angle between two quaternions.

    Parameters:
    q1 (list or tuple): First quaternion represented as [q0, qx, qy, qz]
    q2 (list or tuple): Second quaternion represented as [q0, qx, qy, qz]

    Returns:
    float: Rotation angle between the two quaternions in radians
    """
    # Normalize the quaternions
    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)

    # Compute the dot product
    dot_product = np.dot(q1, q2)

    # Ensure the dot product is within the valid range for arccos
    dot_product = np.clip(dot_product, -1.0, 1.0)

    # Calculate the angle
    angle = 2 * np.arccos(np.abs(dot_product))

    return angle