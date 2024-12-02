import copy
import random

from RANSAC import ransac_rigid_transform, TranslationEstimator, CostumOutlierRemoval
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.linalg import svd
from scipy.linalg import solve
from sklearn.linear_model import RANSACRegressor
from sklearn.linear_model import LinearRegression
import cv2
from numpy.linalg import inv, det
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from Utils import _arrow3D
from Utils import axis_from_rotation_matrix, angle_from_rotation_matrix
import matplotlib.colors as mcolors
from numpy.linalg import inv
from sklearn.linear_model import LinearRegression

def hsv_to_rgba(h, s, v):
    rgb = mcolors.hsv_to_rgb([h, s, v])
    rgba = list(rgb) + [1.0]  # Adding alpha value of 1.0
    return rgba

class DSample:
    def __init__(self):
        self.ref = None
        self.index_1 = None
        self.t_1 = None
        self.target = None
        self.index_2 = None
        self.t_2 = None
        self.valid = False

def delta_rotation_samples(s1, s2, i, j):
    dref = s1['ref'][:3,:3].dot(s2['ref'][:3,:3].T)
    dtarget = s1['target'][:3,:3].dot(s2['target'][:3,:3].T)

    ds = DSample()
    ds.ref = axis_from_rotation_matrix(dref)
    ds.target = axis_from_rotation_matrix(dtarget)

    refA = angle_from_rotation_matrix(dref)
    targetA = angle_from_rotation_matrix(dtarget)
    ds.valid = refA > 0.4 and targetA > 0.4 and np.linalg.norm(ds.ref) > 0.01 and np.linalg.norm(ds.target) > 0.01
    # ds.valid = True

    # ds.ref = ds.ref / np.linalg.norm(ds.ref) * refA
    ds.ref = ds.ref / np.linalg.norm(ds.ref)
    ds.index_1 = i
    # ds.target = ds.target / np.linalg.norm(ds.target) * targetA
    ds.target = ds.target / np.linalg.norm(ds.target)
    ds.index_2 = j
    return ds


def visalize_rots(ref, target, rot):
    assert ref.shape[0] == target.shape[0]
    h = 0.0
    s = 1.0  # You can change this value if you want a different saturation
    v = 1.0
    L = ref.shape[0]
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    angles = []
    for i in range(0, L):
        h = i/L
        color = hsv_to_rgba(h, s, v)
        r = rot @ ref[i].reshape(3)
        t = target[i].reshape(3)
        angle = np.arccos(np.dot(r,t)/(np.linalg.norm(r)*np.linalg.norm(t)))
        angles.append(angle)
        if i % int(L/50) == 0:
            ax.arrow3D(0, 0, 0,
                       r[0], r[1], r[2],
                       mutation_scale=20,
                       arrowstyle="-|>",
                       ec=color,
                       fc='black',
                       linestyle='dashed')
            ax.arrow3D(0, 0, 0,
                       t[0], t[1], t[2],
                       mutation_scale=20,
                       ec=color,
                       fc=color)



    ax.set_title('3D Arrows Demo')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    fig.tight_layout()
    plt.show()

    plt.figure()
    tobehist = np.linalg.norm(angles)/np.sqrt(len(angles))
    plt.hist(angles)
    plt.show()




def best_rotation_transform(r, t):
    """
    Find the best rotation matrix to map points r to points t.

    Parameters:
        r (np.ndarray): Nx3 array of source points.
        t (np.ndarray): Nx3 array of target points.

    Returns:
        error (float): The sum of squared differences (error).
        rotation_matrix (np.ndarray): 3x3 rotation matrix.
        transformed_data (np.ndarray): Rotated version of r.
    """

    # Ensure input arrays are numpy arrays
    r = np.asarray(r)
    t = np.asarray(t)

    # Check the shape of the input arrays
    if r.shape[1] != 3 or t.shape[1] != 3 or r.shape[0] != t.shape[0]:
        raise ValueError("Input arrays must have shape Nx3 and be the same length.")

    # Center the points
    r_centered = r - np.mean(r, axis=0)
    t_centered = t - np.mean(t, axis=0)

    # Compute the covariance matrix
    H = r_centered.T @ t_centered

    # Perform Singular Value Decomposition
    U, S, Vt = np.linalg.svd(H)

    # Calculate the rotation matrix
    rotation_matrix = Vt.T @ U.T

    # Ensure a right-handed coordinate system
    if np.linalg.det(rotation_matrix) < 0:
        Vt[2, :] *= -1
        rotation_matrix = Vt.T @ U.T

    # Rotate the original points
    transformed_data = r @ rotation_matrix.T

    # Calculate the error as the sum of squared differences
    error = np.sqrt(np.sum((transformed_data - t) ** 2))

    return error, rotation_matrix, transformed_data.T


def visualize_rot_path(ref, target, rot):
    r = ref.T
    t = target.T
    error, tscale, t_pred = best_rotation_transform(r.T,t.T)
    r1 = rot@ref.T
    r_norm = np.linalg.norm(r, axis=0)
    t_norm = np.linalg.norm(t, axis=0)
    plt.plot(r_norm, label="ref")
    plt.plot(t_norm, label="tar")
    plt.legend()
    plt.show()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(t_pred[0,:], t_pred[1,:], t_pred[2,:], color="red")
    ax.plot(t[0,:], t[1,:], t[2,:], color="blue", linestyle='dashed')
    ax.plot(r1[0,:], r1[1,:], r1[2,:], color="green", linestyle='dashed')
    print (f"error: {error/t.shape[1]}")
    fig.tight_layout()
    fig.show()


def calibrate_rotation(samples,sampling_ration,apply_ransac, vis):
    deltas = []

    for i in range(0, len(samples), sampling_ration):
        j = len(samples) - i - 1
        delta = delta_rotation_samples(samples[i], samples[j], i, j)
        if delta.valid:
            deltas.append(delta)

    print(f"Got {len(samples)} samples with {len(deltas)} delta samples")

    ref_points = np.array([delta.ref for delta in deltas])

    target_points = np.array([delta.target for delta in deltas])
    inliers = [True for i in range(ref_points.shape[0])]
    # R_matrix, t_vector, inliers = ransac_rigid_transform(ref_points, target_points, residual_threshold=0.2, stop_probability=0.99, stop_n_inliers=100000)
    if apply_ransac:
        R_matrix, t_vector, inliers = ransac_rigid_transform(ref_points, target_points, residual_threshold=100000,stop_probability=0.999999)
        ref_points = ref_points[inliers]
        target_points = target_points[inliers]

    ref_centroid = np.mean(ref_points, axis=0)
    target_centroid = np.mean(target_points, axis=0)

    ref_points_c = ref_points - ref_centroid
    target_points_c = target_points - target_centroid

    cross_cv = ref_points_c.T.dot(target_points_c)

    U, _, Vt = svd(cross_cv)
    V = Vt.T

    i = np.eye(3)
    if np.linalg.det(U.dot(V.T)) < 0:
        i[2, 2] = -1

    rot = V.dot(i).dot(U.T)
    # rot = rot.T
    errors = []
    for k in range(ref_points.shape[0]):
        e = np.linalg.norm(rot @ ref_points[k].reshape(3,1) - target_points[k].reshape(3,1))
        errors.append(e)
    # plt.hist(errors)
    # plt.show()
    err = np.mean(errors)
    print(f"rotation err = {np.mean(errors)}, rotation std = {np.std(errors):0.8f}")

    if vis:
        # visualize_rot_path(ref_points, target_points, rot)
        visalize_rots(ref_points, target_points, rot)

    euler = R.from_matrix(rot).as_euler('zyx', degrees=True)

    # print(rot)
    # print(np.linalg.det(rot))
    # print(f"Calibrated rotation: yaw={euler[0]:.2f} pitch={euler[1]:.2f} roll={euler[2]:.2f}")
    return rot, err, inliers, deltas

def add_axis_noise(axis, std):
    s = np.random.normal(0.0, std, 3)
    axis1 = axis + s
    return axis1

def add_angle_noise(angle, std):
    s = np.random.normal(0.0, std, 1)
    angle1 = angle + s
    return angle1

def add_translation_noise(translation, std):
    s = np.random.normal(0.0, std, 3)
    translation1 = translation + s
    return translation1

def transformation_matrix(axis, angle, translation):
    # Normalize the axis vector
    axis = axis / np.linalg.norm(axis)

    # Compute the rotation matrix using Rodrigues' rotation formula
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0]
    ])
    I = np.eye(3)
    R = I + np.sin(angle) * K + (1 - np.cos(angle)) * np.dot(K, K)

    # Create the transformation matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = translation

    return T

def add_noise(arr):
    axis = axis_from_rotation_matrix(arr)
    angle = angle_from_rotation_matrix(arr)
    translation = arr[:3,3]

    axis1 = add_axis_noise(axis, std = 0.01)
    angle1 = add_angle_noise(angle, std = 0.01)
    translation1 = add_translation_noise(translation, std = 0.1)
    T = transformation_matrix(axis1, angle1, translation1)
    return T



def convert(s: str, noise=False):
    arr = np.zeros((4, 4))
    rows = s.split('\n')[:4]
    for i, r in enumerate(rows):
        cols = r.split('\t')[:4]
        for j, c in enumerate(cols):
            arr[i, j] = float(c)
    if noise:
        arr = add_noise(arr)
    return arr

def calibrate_translation(samples, apply_ransac=True):
    deltas = []

    for i in range(len(samples)):
        j = len(samples) - 1
        QAi = samples[i]["ref"][:3, :3].T
        QAj = samples[j]["ref"][:3, :3].T
        dQA = QAj - QAi
        CA = QAj @ (samples[j]["ref"][:3, 3] - samples[j]["target"][:3, 3]) - QAi @ (samples[i]["ref"][:3, 3] - samples[i]["target"][:3, 3])
        deltas.append((CA, dQA))

        QBi = samples[i]["target"][:3, :3].T
        QBj = samples[j]["target"][:3, :3].T
        dQB = QBj - QBi
        CB = QBj @ (samples[j]["ref"][:3, 3] - samples[j]["target"][:3, 3]) - QBi @ (samples[i]["ref"][:3, 3] - samples[i]["target"][:3, 3])
        deltas.append((CB, dQB))

    constants = np.zeros(len(deltas) * 3)
    coefficients = np.zeros((len(deltas) * 3, 3))

    for i, (CA, dQA) in enumerate(deltas):
        for axis in range(3):
            constants[i * 3 + axis] = CA[axis]
            coefficients[i * 3 + axis, :] = dQA[axis, :]

    A = coefficients
    B = constants
    costumOutlierRemoval = CostumOutlierRemoval(apply_ransac=True)
    trans, trans_error, trans_std = costumOutlierRemoval.fit(A,B)
    # # ransac = RANSACRegressor(TranslationEstimator(), residual_threshold=50.0)
    # ransac = RANSACRegressor(LinearRegression(), residual_threshold=30.0)
    #
    # refs = np.array([s['ref'] for s in samples])
    # targets = np.array([s['target'] for s in samples])
    # ransac.fit(A,B)
    # # ransac.fit(refs,targets)
    #
    # # Get the inliers
    # inlier_mask = ransac.inlier_mask_
    # A_inliers = A[inlier_mask]
    # B_inliers = B[inlier_mask]
    #
    # # Fit the model again using only inliers
    # model = LinearRegression()
    # model.fit(A_inliers, B_inliers)
    # trans = model.coef_
    #
    # trans2 = np.linalg.lstsq(coefficients[inlier_mask], constants[inlier_mask], rcond=None)[0]
    # # trans3 = solve(coefficients, constants)
    #
    # # U, s, Vt = svd(coefficients, full_matrices=False)
    # #
    # # # Compute the pseudo-inverse of the singular values matrix
    # # S_inv = np.diag(1 / s)
    # #
    # # # Solve for the least squares solution
    # # trans4 = Vt.T @ S_inv @ U.T @ constants
    #
    #
    # err = []
    # for i in range(len(coefficients)):
    #     Ax = np.dot(coefficients[i,:] , trans)
    #     a = Ax - constants[i]
    #     err.append(abs(a))
    #     # print(a)
    # err = np.array(err)
    # trans_error = np.mean(err)
    # print(f"translation error = {trans_error}, trans std={np.std(err)}")
    transcm = trans

    print(f"Calibrated translation x={transcm[0]:.2f} y={transcm[1]:.2f} z={transcm[2]:.2f}, error:{trans_error}")
    return trans, trans_error, trans_std

def solve_ax_b_with_ransac(A, B, residual_threshold=10.0):
    """
    Solves the equation Ax = B using RANSAC to remove outliers.

    Parameters:
    A (numpy.ndarray): The matrix A with shape (N, 3).
    B (numpy.ndarray): The matrix B with shape (N, 1).

    Returns:
    numpy.ndarray: The solution vector x after removing outliers using RANSAC.
    """
    # Use RANSAC to fit the model and remove outliers
    ransac = RANSACRegressor(LinearRegression(), residual_threshold=residual_threshold)
    ransac.fit(A, B.ravel())

    # Get the inliers
    inlier_mask = ransac.inlier_mask_
    A_inliers = A[inlier_mask]
    B_inliers = B[inlier_mask]

    # Fit the model again using only inliers
    model = LinearRegression()
    model.fit(A_inliers, B_inliers)
    X_ransac = model.coef_

    return X_ransac, inlier_mask

def make_homogeneous(rotation_matrix):
    homogeneous_matrix = np.eye(4)
    homogeneous_matrix[:3, :3] = rotation_matrix
    return homogeneous_matrix

def rotate_samples(samples, rot):
    new_samples = copy.deepcopy(samples)
    for i, sample in enumerate(samples):
        R1 = sample["target"]
        # R2 = np.linalg.inv(make_homogeneous(rot)) @ R1
        R2 = make_homogeneous(rot.T) @ R1
        new_samples[i]["target"] = R2
    return new_samples

# Q0 = convert(noise=False, s="0.8655022\t0.3619636\t0.346249\t11.01549\t\n0.3356658\t0.09396958\t-0.9372824\t-11.96935\t\n-0.371799\t0.927444\t-0.04016793\t-9.240864\t\n0\t0\t0\t1\t\n")
# R0 = convert(noise=False, s="0.864244\t0.4835299\t-0.1388568\t-7.749919\t\n-0.3799747\t0.8083051\t0.4497355\t-14.12503\t\n0.3296992\t-0.3359192\t0.882302\t-9.50906\t\n0\t0\t0\t1\t\n")
# Q1 = convert(noise=False, s="0.6327188\t-0.6597806\t-0.4054091\t3.185164\t\n-0.6224115\t-0.1218104\t-0.7731535\t-16.78614\t\n0.4607286\t0.7415201\t-0.4877268\t-7.621033\t\n0\t0\t0\t1\t\n")
# R1 = convert(noise=False, s="-0.2949208\t0.5418206\t-0.7870532\t-7.749919\t\n-0.2990566\t0.7299677\t0.6145832\t-14.12503\t\n0.9075173\t0.4166265\t-0.05324769\t-9.50906\t\n0\t0\t0\t1\t\n")
# Q2 = convert(noise=False, s="-0.2933937\t-0.5215114\t0.8012155\t12.69218\t\n0.2654409\t-0.8495843\t-0.455794\t0.2045612\t\n0.9184017\t0.07894827\t0.3876928\t13.74288\t\n0\t0\t0\t1\t\n")
# R2 = convert(noise=False, s="-0.3691924\t-0.476792\t-0.7977262\t-7.749919\t\n0.7123106\t0.4061498\t-0.5724127\t-14.12503\t\n0.5969182\t-0.7795589\t0.1896763\t-9.50906\t\n0\t0\t0\t1\t\n")
# Q3 = convert(noise=False, s="0.6396689\t0.7565885\t0.1356382\t3.497107\t\n-0.2200595\t0.01118261\t0.9754225\t12.99156\t\n0.7364765\t-0.653796\t0.1736476\t12.9996\t\n0\t0\t0\t1\t\n")
# R3 = convert(noise=False, s="0.5003181\t-0.4941545\t0.7109805\t-7.749919\t\n0.02558181\t-0.8123491\t-0.5826108\t-14.12503\t\n0.8654639\t0.309679\t-0.3937918\t-9.50906\t\n0\t0\t0\t1\t\n")
# Q4 = convert(noise=False, s="-0.3149856\t0.9490425\t-0.01013368\t-8.39291\t\n0.6556992\t0.2098811\t-0.7252646\t-6.500123\t\n-0.68618\t-0.2350925\t-0.688396\t-15.40479\t\n0\t0\t0\t1\t\n")
# R4 = convert(noise=False, s="0.4248509\t0.5857432\t0.6902223\t-7.749919\t\n-0.1650455\t0.799794\t-0.5771391\t-14.12503\t\n-0.890091\t0.1312801\t0.4364674\t-9.50906\t\n0\t0\t0\t1\t\n")
#
# H0 = convert(noise=False, s="0.9750824\t-0.1413145\t0.1710101\t0\t\n0.1710101\t0.9698463\t-0.1736483\t0\t\n-0.1413145\t0.1985658\t0.9698463\t0\t\n0\t0\t0\t1\t\n")
# H1 = convert(noise=False, s="0.3723454\t0.325589\t-0.8691092\t0\t\n-0.7641872\t0.6389597\t-0.088025\t0\t\n0.5266658\t0.6969378\t0.4867247\t0\t\n0\t0\t0\t1\t\n")
# H2 = convert(noise=False, s="-0.2573245\t-0.9287764\t-0.2667563\t0\t\n-0.03237873\t0.2841843\t-0.9582229\t0\t\n0.9657826\t-0.2379368\t-0.1032003\t0\t\n0\t0\t0\t1\t\n")
# H3 = convert(noise=False, s="0.8150225\t0.1213478\t0.5665802\t0\t\n-0.02873431\t-0.9681571\t0.24869\t0\t\n0.5787165\t-0.2189682\t-0.785583\t0\t\n0\t0\t0\t1\t\n")
# H4 = convert(noise=False, s="-0.07648528\t0.1817485\t0.9803661\t0\t\n0.5426667\t0.832449\t-0.1119892\t0\t\n-0.8364586\t0.5234464\t-0.1622989\t0\t\n0\t0\t0\t1\t\n")
#
# samples = [
#     {'ref': Q0, 'target': R0},
#     {'ref': Q1, 'target': R1},
#     {'ref': Q2, 'target': R2},
#     {'ref': Q3, 'target': R3},
#     {'ref': Q4, 'target': R4}
# ]

def read_from_txt(txt_file):
    samples = []
    with open(txt_file, 'r') as f:
        lines = f.readlines()
        for i in range(0,len(lines)-1, 2):
            ref_mat = np.reshape(np.asarray([float(l) for l in lines[i].split(',')[:-1]]),(4,4))
            tar_mat = np.reshape(np.asarray([float(l) for l in lines[i+1].split(',')[:-1]]),(4,4))
            samples.append({'ref':ref_mat, 'target':tar_mat})
    return samples
def find_T(samples, F):
    R1 = samples[10]['ref']
    Q1 = samples[10]['target']
    h = inv(F) @ R1
    P = inv(h) @ Q1
    print(P)
    return P

if __name__ == "__main__":
    samples = read_from_txt("F:\work\ARassis\\tracking\calibration\CalibrationTest\Assets\\tracks.txt")
    rot, err, inliers, deltas= calibrate_rotation(samples, vis=False, sampling_ration=1 , apply_ransac=False)
    print(rot)
    a = 0
    rotated_samples = rotate_samples(samples, rot)
    trans, trans_err, tras_std = calibrate_translation(rotated_samples)
    print(rot)

"""
F
0.75063	-0.40782	0.51984	0.00000
0.64086	0.64086	-0.42262	0.00000
-0.16079	0.65037	0.74240	0.00000
0.00000	0.00000	0.00000	1.00000

tracker to hololens
0.95388	0.23795	0.18301	10.00000
0.12941	0.22414	-0.96593	-15.00000
-0.27087	0.94506	0.18301	-5.00000
0.00000	0.00000	0.00000	1.00000
"""
# F = np.eye(4)
# F[:3,:3] = rot.T
# F[:3, 3] = trans
# # print(Q0)
# # print(R0)
# h = F @ R1
# P = inv(h) @ Q1
# print("\n")
# print("P")
# print(P)
# a=0


# for s in samples:
#     # print(s)
#     q = s['ref']
#     r = s['target']
#
#     h = F @ q[:4, :4]
#     P = r[:4, :4].T @ h
#     print(P)
