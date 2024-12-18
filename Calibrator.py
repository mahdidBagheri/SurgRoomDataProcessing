import copy

from Config import Config
from Config.CalibrationConfig import rotation_residual,translation_residual
from RANSAC import ransac_rigid_transform, CostumOutlierRemoval, ransac_mean_transformation
import numpy as np
from scipy.linalg import svd
from sklearn.linear_model import RANSACRegressor
from numpy.linalg import inv
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from Utils import axis_from_rotation_matrix, angle_from_rotation_matrix
import matplotlib.colors as mcolors
from sklearn.linear_model import LinearRegression

def hsv_to_rgba(h, s, v):
    rgb = mcolors.hsv_to_rgb([h, s, v])
    rgba = list(rgb) + [1.0]  # Adding alpha value of 1.0
    return rgba

class DSample:
    def __init__(self):
        self.ref = None
        self.ref_angle = None
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
    ds.ref_angle = refA
    # ds.ref = ds.ref / np.linalg.norm(ds.ref) * refA
    ds.ref = ds.ref / np.linalg.norm(ds.ref)
    ds.index_1 = i
    # ds.target = ds.target / np.linalg.norm(ds.target) * targetA
    ds.target = ds.target / np.linalg.norm(ds.target)
    ds.index_2 = j

    return ds

def visalize_rots(ref, target, rot, n=20):
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
        if i % int(L/n) == 0:
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

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(t_pred[0,:], t_pred[1,:], t_pred[2,:], color="red")
    ax.plot(t[0,:], t[1,:], t[2,:], color="blue", linestyle='dashed')
    ax.plot(r1[0,:], r1[1,:], r1[2,:], color="green", linestyle='dashed')
    print (f"error: {error/t.shape[1]}")
    fig.tight_layout()
    plt.show(block=False)


def calibrate_rotation(samples,apply_ransac,init_deltas, vis):
    deltas = [] + init_deltas

    for i in range(0, len(samples)):
        j = len(samples) - i - 1
        delta = delta_rotation_samples(samples[i], samples[j], i, j)
        if delta.valid:
            deltas.append(delta)


    print(f"Got {len(samples)} samples with {len(deltas)} delta samples")

    ref_points = np.array([delta.ref for delta in deltas])

    target_points = np.array([delta.target for delta in deltas])
    inliers = [True for i in range(ref_points.shape[0])]
    if apply_ransac:
        R_matrix, t_vector, inliers = ransac_rigid_transform(ref_points, target_points, residual_threshold=rotation_residual,stop_probability=0.9999)
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
    errors = []
    for k in range(ref_points.shape[0]):
        e = np.linalg.norm(rot @ ref_points[k].reshape(3,1) - target_points[k].reshape(3,1))
        errors.append(e)

    err = np.mean(errors)
    print(f"rotation err = {np.mean(errors)}, rotation std = {np.std(errors):0.8f}")

    if vis:
        visualize_rot_path(ref_points, target_points, rot)
        visalize_rots(ref_points, target_points, rot)

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

def calibrate_translation(samples, rot, apply_ransac=True):
    rotated_samples = rotate_samples(samples, rot)
    deltas = []

    for i in range(len(rotated_samples)):
        j = len(rotated_samples) - i - 1
        QAi = rotated_samples[i]["ref"][:3, :3].T
        QAj = rotated_samples[j]["ref"][:3, :3].T
        dQA = QAj - QAi
        CA = QAj @ (rotated_samples[j]["ref"][:3, 3] - rotated_samples[j]["target"][:3, 3]) - QAi @ (rotated_samples[i]["ref"][:3, 3] - rotated_samples[i]["target"][:3, 3])
        deltas.append((CA, dQA))

        QBi = rotated_samples[i]["target"][:3, :3].T
        QBj = rotated_samples[j]["target"][:3, :3].T
        dQB = QBj - QBi
        CB = QBj @ (rotated_samples[j]["ref"][:3, 3] - rotated_samples[j]["target"][:3, 3]) - QBi @ (rotated_samples[i]["ref"][:3, 3] - rotated_samples[i]["target"][:3, 3])
        deltas.append((CB, dQB))

    constants = np.zeros(len(deltas) * 3)
    coefficients = np.zeros((len(deltas) * 3, 3))

    for i, (CA, dQA) in enumerate(deltas):
        for axis in range(3):
            constants[i * 3 + axis] = CA[axis]
            coefficients[i * 3 + axis, :] = dQA[axis, :]

    A = coefficients
    B = constants
    costumOutlierRemoval = CostumOutlierRemoval(apply_ransac=True, e_thresh=translation_residual)
    trans, trans_error, trans_std = costumOutlierRemoval.fit(A,B)
    transcm = trans

    print(f"Calibrated translation x={transcm[0]:.2f} y={transcm[1]:.2f} z={transcm[2]:.2f}, error:{trans_error:.2f}")
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

def read_from_txt(txt_file):
    samples = []
    with open(txt_file, 'r') as f:
        lines = f.readlines()
        for i in range(0,len(lines)-1, 2):
            ref_mat = np.reshape(np.asarray([float(l) for l in lines[i].split(',')[:-1]]),(4,4))
            tar_mat = np.reshape(np.asarray([float(l) for l in lines[i+1].split(',')[:-1]]),(4,4))
            samples.append({'ref':ref_mat, 'target':tar_mat})
    return samples


def find_T(samples, F, threshold = 1):
    preds = []
    for s in range(len(samples)):
        R1 = samples[s]['ref']
        Q1 = samples[s]['target']
        h = inv(F) @ R1
        P = inv(h) @ Q1
        preds.append(P)
    mean_transformation = ransac_mean_transformation(preds, threshold)

    return mean_transformation

if __name__ == "__main__":
    samples = read_from_txt("F:\work\ARassis\\tracking\calibration\CalibrationTest\Assets\\tracks.txt")
    rot, err, inliers, deltas= calibrate_rotation(samples, vis=False , apply_ransac=False)
    print(rot)
    a = 0
    rotated_samples = rotate_samples(samples, rot)
    trans, trans_err, tras_std = calibrate_translation(rotated_samples)
    print(rot)
