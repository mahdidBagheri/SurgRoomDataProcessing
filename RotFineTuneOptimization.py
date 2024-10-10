import copy
import math
import warnings

from numpy.linalg import inv
from scipy.spatial.transform import Rotation as R
warnings.simplefilter(action='ignore', category=FutureWarning)
warnings.simplefilter(action='ignore', category=DeprecationWarning)
from Utils import quaternion_rotation_angle_between

import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from RANSAC import ransac_rigid_transform
from matplotlib import cm
from Calibrator import calibrate_rotation, rotate_samples, calibrate_translation
column_list = ["dQ0", "dQx", "dQy", "dQz", "dQ", "dTx", "dTy", "dTz", "dT"]
org_column_list = ["Q0", "Qx", "Qy", "Qz", "Tx", "Ty", "Tz"]




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



def dot_product(arr1, arr2):
    max_len = max(len(arr1), len(arr2))
    arr1_extended = np.concatenate((arr1 , [0.0] * (max_len - len(arr1))), axis=0)
    arr2_extended = np.concatenate((arr2 , [0.0] * (max_len - len(arr2))), axis=0)
    dot_product_result = arr1_extended * arr2_extended
    return dot_product_result


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

def plot_speeds(ex_div,ex_dt, hl_div,hl_dt,intersection):
    plt.figure()
    plt.plot(ex_div["index"], ex_div["dT"], color="red", label="ex")
    plt.plot( ex_dt, color="purple", label="ex_dt")
    plt.plot(hl_div["index"], hl_div["dT"], color="blue", label="hl")
    plt.plot(hl_dt, color="green", label="hl_dt")
    plt.plot(intersection, color="black", label="intersect")
    plt.legend()
    plt.show()


def find_std_in_angles(t_hl, t_ex):
    pass


def find_samples(one_piles, shift, coef):
    samples = []
    A = []
    B = []
    for pile in one_piles:
        i1 = pile[0]
        i2 = pile[1]

        index = int((i1 + i2)/2)
        i1 = (i2 - i1)*0.8 + i1
        # i2 = (i2 - i1)*0.7 + i1
        hl_index = int(index)
        hl_index_s = int(i1)
        hl_index_e = int(i2)
        ex_index = int((index+shift)/coef)
        ex_index_s = int((i1+shift)/coef)
        ex_index_e = int((i2+shift)/coef)
        # t_ex = ex.loc[ex_index]
        # t_hl = hl.loc[hl_index]

        t_ex = ex.loc[ex_index_s:ex_index_e]
        t_hl = hl.loc[hl_index_s:hl_index_e]
        # print(f"ex:{t_ex}\nhl:{t_hl}")
        find_std_in_angles(t_hl, t_ex)
        M_ex = create_transformation_matrix([t_ex["Q0"].mean(),t_ex["Qx"].mean(),t_ex["Qy"].mean(),t_ex["Qz"].mean()], [t_ex['Tx'].mean(),t_ex['Ty'].mean(),t_ex['Tz'].mean() ])
        M_hl = create_transformation_matrix([t_hl["Q0"].mean(),t_hl["Qx"].mean(),t_hl["Qy"].mean(),t_hl["Qz"].mean()], [t_hl['Tx'].mean(),t_hl['Ty'].mean(),t_hl['Tz'].mean() ])
        A.append(M_ex)
        B.append(M_hl)
        samples.append({'ref':M_ex, 'target':M_hl})
    a = 0
    return samples

def steady_mask(ex_div, hl_div):
    ex_dt = np.zeros(int(np.max(ex_div["index"]))+1)
    for i in range(len(ex_div)-1):
        if ex_div.loc[i, 'dT'] < 0.30:
            ex_dt[int(ex_div.loc[i,"index"])] = 1.0
            if (i < len(ex_div)):
                ex_dt[int(ex_div.loc[i,"index"])+1] = 1.0

    hl_dt = np.zeros(int(np.max(hl_div["index"]))+1)
    for i in range(len(hl_div)-1):
        if hl_div.loc[i, 'dT'] < 0.30:
            hl_dt[int(hl_div.loc[i,"index"])] = 1.0

    return ex_dt, hl_dt


def is_rotation_matrix(R):
    Rt = np.transpose(R)
    should_be_identity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - should_be_identity)
    return n < 1e-6


def rotation_matrix_to_euler_angles(R):
    assert (is_rotation_matrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


def angle_between_rotation_matrices(R1, R2):
    # Ensure the input matrices are valid rotation matrices
    assert R1.shape == (3, 3) and R2.shape == (3, 3), "Input matrices must be 3x3."

    # Compute the relative rotation matrix
    R_diff = np.dot(R1, R2.T)

    # Convert the relative rotation matrix to a rotation object
    rotation_diff = R.from_matrix(R_diff)

    # Extract the angle from the rotation object
    angle = rotation_diff.magnitude()

    return angle

def test(samples, rot, trans):
    F = np.eye(4)
    F[:3, :3] = rot.T
    F[:3, 3] = trans

    print(F)
    ds = []
    for s in samples:
        Q1 = s['ref']
        R1 = s['target']
        d = angle_between_rotation_matrices((F @ Q1)[:3,:3], R1[:3,:3])
        print("d")
        print (d)
        ds.append(d)

        h = F @ R1
        P = inv(h) @ Q1
        print("\n")
        print("P")
        print(P)
        a = 0
        # h = F @ Q1[:3, :3]
        # P = R1[:3, :3].T @ h
        # print("P")
        # print(rotation_matrix_to_euler_angles(P))
        # print("\n")
    plt.hist(ds)
    plt.show()
    m = np.mean(ds)
    s = np.std(ds)
    print(f"mean {m}, std {s}")
    inliers = np.logical_and(ds < m+s*0.65 , ds > m-s*0.65)
    ds = np.asarray(ds)
    new_ds = ds[inliers]
    print(f"new ds mean {np.mean(new_ds)}, new ds std {np.std(new_ds)}")
    precetage_of_inliers = np.sum(inliers) / inliers.size
    print(f"percetage of inliers in final rotations = {precetage_of_inliers}")
    plt.hist(new_ds)
    plt.show()

def find_correspondance_positions(ex, hl, shift, coef):
    A = []
    B = []
    for i in range(0,len(hl),1):
        j = int((i + shift) / coef)
        if j > 0 and j < len(ex) - 1:
            hl_pose = [hl.loc[i,"Tx"], hl.loc[i,"Ty"], hl.loc[i,"Tz"]]
            ex_pose = [ex.loc[j,"Tx"], ex.loc[j,"Ty"], ex.loc[j,"Tz"]]
            if not (np.isnan(ex_pose[0]) or np.isnan(ex_pose[1]) or np.isnan(ex_pose[2])):
                A.append(ex_pose)
                B.append(hl_pose)
    return A, B

def rigid_transform_3D(A, B):
    assert len(A) == len(B)

    N = A.shape[0]  # Total points

    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    # Center the points
    AA = A - centroid_A
    BB = B - centroid_B

    # Dot is matrix multiplication for array
    H = np.dot(AA.T, BB)

    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    # Special reflection case
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = np.dot(Vt.T, U.T)

    t = centroid_B.T - np.dot(R, centroid_A.T)

    M = np.eye(4)
    M[:3,:3] = R
    M[:3,3] = t

    errors = []
    for a,b in zip(A,B):
        Ma = M @ np.array([[a[0]],[a[1]],[a[2]],[1.0]])
        err = np.array([Ma[0], Ma[1], Ma[2]]) - b.reshape(3,1)
        errors.append(err)

    mean_error = np.mean(errors)
    # print(f"mean error:{mean_error}")
    return R, t


def transform_3D(A, R, t):
    transformed_a = []
    M = np.eye(4)
    M[:3,:3] = R
    M[:3,3] = t
    for a in A:
        new_a = M @ np.array([[a[0]],[a[1]],[a[2]],[1.0]])
        transformed_a.append([float(new_a[0]), float(new_a[1]), float(new_a[2])])
    return np.asarray(transformed_a)

n = 5
ex = pd.read_csv(f"ex_data{n}.csv")
hl = pd.read_csv(f"hl_data{n}.csv")

df = pd.DataFrame(columns=["shift", "coef", "mean"])
# SHIFT = range(138,150,1)

def calc_deltas(points):
    deltas = []
    for i in range(int(len(points)/2)):
        q1 = [points.iloc[i+0]["Q0"], points.iloc[i+0]["Qx"], points.iloc[i+0]["Qy"], points.iloc[i+0]["Qz"]]
        q2 = [points.iloc[i+1]["Q0"], points.iloc[i+1]["Qx"], points.iloc[i+1]["Qy"], points.iloc[i+1]["Qz"]]
        a = quaternion_rotation_angle_between(q1, q2)
        deltas.append(float(a))
    return np.asarray(deltas)


if n == 3:
    SHIFT = [149] # for n = 3
    # SHIFT = range(50,250,1)
    COEF = [1.506533] # for n = 3
    # COEF = np.linspace(1.4, 1.6, 200)
    INTERSECTION_THRESH = 30

elif n == 4:
    SHIFT = [600+56-190-120] # for n = 4
    COEF = [0.66] # for n = 4
    INTERSECTION_THRESH = 100

elif n == 5:
    # SHIFT = [-1020] # for n = 5
    SHIFT = range(850,1100,1)
    # COEF = [0.505] # for n = 5
    COEF = np.linspace(0.4, 0.6, 200)
    INTERSECTION_THRESH = 100

elif n == 6:
    # SHIFT = [-948] # for n = 6
    SHIFT = range(-1000,-800,10) # for n = 6
    # COEF = [0.49321608040201] # for n = 6
    COEF = np.linspace(0.45,0.55,10)



elif n == 7:
    SHIFT = [-0] # for n = 7
    COEF = [0.505] # for n = 7
    INTERSECTION_THRESH = 100


hl_random_points = np.random.randint(0, int(len(hl)*0.6), 500)
hl_points = hl.iloc[hl_random_points]
hl_angles = calc_deltas(hl_points)
clmns =["shift", "coef", "loss"]
df = pd.DataFrame(columns=clmns)
loss_dict = {}
for shift in SHIFT:
    for coef in COEF:
        ex_random_points = np.round((hl_random_points+shift)/coef).astype(int)
        ex_points = ex.iloc[ex_random_points]
        ex_angles = calc_deltas(ex_points)
        nonenans = np.logical_not(np.isnan(ex_angles))
        loss = np.linalg.norm(ex_angles[nonenans] - hl_angles[nonenans])/len(hl_angles)
        loss_dict = {"shift":float(shift), "coef":float(coef), "loss":float(loss)}
        new_df = pd.DataFrame.from_dict(loss_dict, orient="index")
        df = df._append(loss_dict, ignore_index=True)
        print(f"shift:{shift:0.3f}, coef:{coef:0.3f}, loss:{loss:0.3f}")

df.to_csv(f"loss{n}.csv")

        ### ######################## ###############









