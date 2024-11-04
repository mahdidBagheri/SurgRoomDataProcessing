import pandas as pd
from Utils import quaternion_to_matrix, axis_from_rotation_matrix
import numpy as np
from scipy.spatial.transform import Rotation as R

def find_rotation_between_vectors(v1, v2):
    # Normalize the vectors
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)

    # Calculate the cross product and dot product
    cross_product = np.cross(v1, v2)
    dot_product = np.dot(v1, v2)

    # Calculate the angle between the vectors
    angle = np.arccos(dot_product)

    # Create the quaternion
    quat = np.concatenate(([np.cos(angle / 2)], np.sin(angle / 2) * cross_product))
    quat = quat / np.linalg.norm(quat)  # Normalize the quaternion

    return R.from_quat(quat).as_matrix()

# data = pd.read_csv("combined_data.csv")
# res_df = pd.DataFrame()
# for i in range(len(data)-2):
#     try:
#         hl1 = [data.loc[i,"hl_Q0"],data.loc[i,"hl_Qx"],data.loc[i,"hl_Qy"],data.loc[i,"hl_Qz"]]
#         hl2 = [data.loc[i+1,"hl_Q0"],data.loc[i+1,"hl_Qx"],data.loc[i+1,"hl_Qy"],data.loc[i+1,"hl_Qz"]]
#         hl1_mat = quaternion_to_matrix(hl1)
#         hl2_mat = quaternion_to_matrix(hl2)
#
#         hl12_mat = hl1_mat @ hl2_mat.T
#         hl12_axis = axis_from_rotation_matrix(hl12_mat)
#
#         ex1 = [data.loc[i,"ex_Q0"],data.loc[i,"ex_Qx"],data.loc[i,"ex_Qy"],data.loc[i,"ex_Qz"]]
#         ex2 = [data.loc[i+1,"ex_Q0"],data.loc[i+1,"ex_Qx"],data.loc[i+1,"ex_Qy"],data.loc[i+1,"ex_Qz"]]
#
#         ex1_mat = quaternion_to_matrix(ex1)
#         ex2_mat = quaternion_to_matrix(ex2)
#
#         ex12_mat = ex1_mat @ ex2_mat.T
#         ex12_axis = axis_from_rotation_matrix(ex12_mat)
#
#         m = find_rotation_between_vectors(hl12_axis, ex12_axis)
#         print(m.flatten())
#     except:
#         print("zero")

