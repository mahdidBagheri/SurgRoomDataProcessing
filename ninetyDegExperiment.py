import math
import numpy as np
import pandas as pd
import matplotlib

from Utils import angle_from_rotation_matrix, create_transformation_matrix
from Utils import axis_from_rotation_matrix

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
column_list = ["dQ0", "dQx", "dQy", "dQz", "dQ", "dTx", "dTy", "dTz", "dT"]
org_column_list = ["Q0", "Qx", "Qy", "Qz", "Tx", "Ty", "Tz"]

def read_hololense_data(n):
    df = pd.DataFrame(columns=["index", "State","Q0","Qx","Qy","Qz","Tx","Ty","Tz"])

    with open(f"{n}.txt", "r") as hl_data:
        lines = hl_data.readlines()
        for i, line in enumerate(lines):
            content = line.split(",")
            data = []

            for d in content[1:]:
                data.append(float(d))
            data_np = np.array(data).reshape((4,4))
            q, t = extract_quaternion_and_translation(data_np)
            new_dict = {"index":i,"State":"OK","Q0":q[0], "Qx":q[1], "Qy":q[2], "Qz":q[3], "Tx":t[0]*1000, "Ty":t[1]*1000, "Tz":t[2]*1000}
            new_df = pd.DataFrame([new_dict])
            df = pd.concat([df, new_df], ignore_index=True)
    return df

def extract_quaternion_and_translation(matrix):
    # Ensure the matrix is 4x4
    assert matrix.shape == (4, 4), "Matrix must be 4x4"

    # Extract the rotation part (upper-left 3x3 submatrix)
    rotation_matrix = matrix[:3, :3]

    # Extract the translation part (last column, first three elements)
    translation_vector = matrix[:3, 3]

    # Convert the rotation matrix to a quaternion
    rotation = R.from_matrix(rotation_matrix)
    quaternion = rotation.as_quat()

    return quaternion, translation_vector

def create_div_mat(df, hl, SHIFT ,COEF ):
    if hl:
        coef = 1
        shift = 0
    else:
        coef = COEF
        shift = SHIFT
    div_df = pd.DataFrame(columns=["index"] + column_list)

    for i in range(df.shape[0]):
        if i == 0:
            new_dict = {"index":i*coef-shift,"dQ0":np.nan, "dQx":np.nan, "dQy":np.nan, "dQz":np.nan, "dTx":np.nan, "dTy":np.nan, "dTz":np.nan}
            # new_dict = {"index":i,"dQ0":np.nan, "dQx":np.nan, "dQy":np.nan, "dQz":np.nan, "dTx":np.nan, "dTy":np.nan, "dTz":np.nan}

        else:
            new_dict = {"index":i*coef-shift}
            # new_dict = {"index":i}
            for c in org_column_list:
                v1 = df.loc[i-1,c]
                v2 = df.loc[i,c]
                dv = v2 - v1
                new_dict.update({"d"+c:dv})

        if (not (np.isnan(new_dict["dTx"]) or np.isnan(new_dict["dTx"]) or np.isnan(new_dict["dTx"]))):
            new_dict.update({"dT":math.sqrt(new_dict["dTx"]**2 + new_dict["dTy"]**2 + new_dict["dTz"]**2)})
        else:
            new_dict.update({"dT":None})

        if (not (np.isnan(new_dict["dQx"]) or np.isnan(new_dict["dQx"]) or np.isnan(new_dict["dQx"]))):
            new_dict.update({"dQ": math.sqrt(new_dict["dQ0"]**2 + new_dict["dQx"]**2 + new_dict["dQy"]**2 + new_dict["dQz"]**2)})
        else:
            new_dict.update({"dQ":None})

        _df = pd.DataFrame([new_dict])
        div_df = pd.concat([div_df, _df], ignore_index=True)

    return div_df

def steadyMask(hl_div):
    hl_dt = np.zeros(int(np.max(hl_div["index"]))+1)
    for i in range(len(hl_div)-1):
        if hl_div.loc[i, 'dT'] < 7.0:
            hl_dt[int(hl_div.loc[i,"index"])] = 1.0

    return hl_dt

def extract_one_piles_indexes(arr, thresh):
    one_piles = []
    start = None

    for i, val in enumerate(arr):
        if val == 1 and start is None:
            start = i
        elif val == 0 and start is not None:
            if(i - start > thresh):
                one_piles.append((start, i - 1))
            start = None

    if start is not None:
        one_piles.append((start, len(arr) - 1))

    return one_piles

def find_samples(hl_df, one_piles):
    samples = []
    A = []
    B = []
    for pile in one_piles:
        i1 = pile[0]
        i2 = pile[1]

        hl_index_s = int(i1)
        hl_index_e = int(i2)
        t_hl = hl_df.loc[hl_index_s:hl_index_e]
        M_hl = create_transformation_matrix([t_hl["Q0"].mean(),t_hl["Qx"].mean(),t_hl["Qy"].mean(),t_hl["Qz"].mean()], [t_hl['Tx'].mean(),t_hl['Ty'].mean(),t_hl['Tz'].mean() ])

        B.append(M_hl)

    return B

def extract_axis_angles(samples):
    axises = []
    angles = []
    for s in samples:
        angle = angle_from_rotation_matrix(s[:3,:3])
        angles.append(angle)
        axis = axis_from_rotation_matrix(s[:3,:3])
        axises.append(axis)

    print(angles)
    print(axises)

    a = 0

def plot_speeds(ex_div,hl_dt):
    plt.figure()
    plt.plot(ex_div["index"], ex_div["dT"], color="red", label="ex")
    # plt.plot(hl_dt, color="green", label="hl_dt")
    plt.legend()
    plt.show()


n = "rot90exp2"
hl_df = read_hololense_data(n)
div_df = create_div_mat(hl_df, hl=1, COEF=1, SHIFT=0)
steady_mask = steadyMask(div_df)
plot_speeds(div_df, div_df)
one_piles = extract_one_piles_indexes(steady_mask,thresh = 150)
samples = find_samples(hl_df, one_piles)
extract_axis_angles(samples)




