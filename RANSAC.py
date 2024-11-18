import math
import random

import numpy as np
from sklearn.linear_model import RANSACRegressor
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.spatial.transform import Rotation as R
from sklearn.base import BaseEstimator, RegressorMixin
from sklearn.linear_model import RANSACRegressor

class RotationEstimator(BaseEstimator, RegressorMixin):
    """
    Custom estimator that finds the best rotation matrix to align two sets of 3D points.
    This estimator assumes the input to `fit` are two sets of 3D points, and the goal is to
    find the optimal rotation matrix that aligns the first set (A) to the second set (B).
    """

    def fit(self, A, B):
        """
        Fit the rotation estimator using the input points A and B.
        A and B must be of shape (n_samples, 3).

        Parameters:
            A (ndarray): Source points, shape (n_samples, 3).
            B (ndarray): Target points, shape (n_samples, 3).

        Returns:
            self: Fitted estimator.
        """
        assert A.shape == B.shape, "Input points must have the same shape."

        # Center the points to remove translation
        A_centered = A - np.mean(A, axis=0)
        B_centered = B - np.mean(B, axis=0)

        # Singular Value Decomposition (SVD) to find optimal rotation
        H = A_centered.T @ B_centered
        U, S, Vt = np.linalg.svd(H)

        # Calculate the rotation matrix
        self.rotation_matrix = Vt.T @ U.T

        # Ensure proper rotation (determinant = 1)
        if np.linalg.det(self.rotation_matrix) < 0:
            Vt[-1, :] = -1
            self.rotation_matrix = Vt.T @ U.T

        return self

    def predict(self, A):
        """
        Apply the estimated rotation to the points A.

        Parameters:
            A (ndarray): Points to be rotated, shape (n_samples, 3).

        Returns:
            ndarray: Rotated points.
        """
        return A @ self.rotation_matrix.T

    def score(self, A, B):
        """
        Score the estimator by computing the negative mean squared error.

        Parameters:
            A (ndarray): Source points, shape (n_samples, 3).
            B (ndarray): Target points, shape (n_samples, 3).

        Returns:
            float: Negative mean squared error.
        """
        A_rotated = self.predict(A)
        return -np.mean(np.linalg.norm(A_rotated - B, axis=1)*2)

class TranslationEstimator(BaseEstimator, RegressorMixin):
    """
    Custom estimator that finds the best rotation matrix to align two sets of 3D points.
    This estimator assumes the input to `fit` are two sets of 3D points, and the goal is to
    find the optimal rotation matrix that aligns the first set (A) to the second set (B).
    """

    def fit(self, ref, target):
        deltas = []
        for i in range(ref.shape[0]):
            for j in range(i):
                QAi = ref[i][:3, :3].T
                QAj = ref[j][:3, :3].T
                dQA = QAj - QAi
                CA = QAj @ (ref[j][:3, 3] - target[j][:3, 3]) - QAi @ (
                            ref[i][:3, 3] - target[i][:3, 3])
                deltas.append((CA, dQA))

                QBi = target[i][:3, :3].T
                QBj = target[j][:3, :3].T
                dQB = QBj - QBi
                CB = QBj @ (ref[j][:3, 3] - target[j][:3, 3]) - QBi @ (
                            ref[i][:3, 3] - target[i][:3, 3])
                deltas.append((CB, dQB))

        constants = np.zeros(len(deltas) * 3)
        coefficients = np.zeros((len(deltas) * 3, 3))

        for i, (CA, dQA) in enumerate(deltas):
            for axis in range(3):
                constants[i * 3 + axis] = CA[axis]
                coefficients[i * 3 + axis, :] = dQA[axis, :]

        self.translation = np.linalg.lstsq(coefficients, constants, rcond=None)[0]

        return self

    def predict(self, A):
        """
        Apply the estimated rotation to the points A.

        Parameters:
            A (ndarray): Points to be rotated, shape (n_samples, 3).

        Returns:
            ndarray: Rotated points.
        """
        return A + self.translation

    def score(self, A, B):
        """
        Score the estimator by computing the negative mean squared error.

        Parameters:
            A (ndarray): Source points, shape (n_samples, 3).
            B (ndarray): Target points, shape (n_samples, 3).

        Returns:
            float: Negative mean squared error.
        """
        A_translated = self.predict(A)
        return -np.mean(np.linalg.norm(A_translated - B, axis=1)*2)

def ransac_rigid_transform(src, dst, vis=False,residual_threshold=7.0, stop_probability=0.99999, stop_n_inliers=4):
    def estimate_rigid_transform(src, dst):
        assert src.shape == dst.shape
        centroid_src = np.mean(src, axis=0)
        centroid_dst = np.mean(dst, axis=0)
        src_centered = src - centroid_src
        dst_centered = dst - centroid_dst
        H = np.dot(src_centered.T, dst_centered)
        U, S, Vt = np.linalg.svd(H)
        R_matrix = np.dot(Vt.T, U.T)
        if np.linalg.det(R_matrix) < 0:
            Vt[-1, :] *= -1
            R_matrix = np.dot(Vt.T, U.T)
        t_vector = centroid_dst.T - np.dot(R_matrix, centroid_src.T)
        return R_matrix, t_vector

    # Perform RANSAC
    estimator = RotationEstimator()

    # Initialize RANSACRegressor with custom estimator
    ransac = RANSACRegressor(estimator=estimator,residual_threshold=0.4, min_samples=3)

    # ransac = RANSACRegressor(residual_threshold=residual_threshold, stop_probability=stop_probability, stop_n_inliers=stop_n_inliers)
    ransac.fit(src, dst)
    inliers = ransac.inlier_mask_
    # inliers = [True for i in range(src.shape[0])]
    precetage_of_inliers = np.sum(inliers) / len(inliers)
    print(f"percetage of inliers in rotation = {precetage_of_inliers}")
    # Estimate rigid transformation using inliers
    R_matrix, t_vector = estimate_rigid_transform(src[inliers], dst[inliers])

    if vis:
        # Visualize the results
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(src[:, 0], src[:, 1], src[:, 2], color='blue', label='Source points')
        ax.scatter(dst[:, 0], dst[:, 1], dst[:, 2], color='red', label='Destination points')
        # ax.scatter(dst[inliers, 0], dst[inliers, 1], dst[inliers, 2], edgecolor='yellow', facecolor='none', s=100, label='Inliers')
        ax.legend()
        plt.show()

    return R_matrix, t_vector, inliers

class CostumOutlierRemoval():
    def __init__(self, apply_ransac, e_thresh=1.0):
        self.w = 0.2
        self.p = 0.99
        self.DoF = 3
        self.thresh = 5.0
        self.apply_ransac = apply_ransac
        self.e_thresh = e_thresh


    def fit(self,A, B):
        results = []
        errors = []
        assert A.shape[0] == B.shape[0]
        l = A.shape[0]
        N = int(np.log(1-self.p)/np.log(1-self.w**self.DoF))*6
        for i in range(N):
            random_selections = self.generate_unique_random_numbers(self.DoF, 0, l)

            sub_A = A[random_selections]
            sub_B = B[random_selections]
            trans = np.linalg.lstsq(sub_A, sub_B, rcond=None)[0]

            e = abs(A @ trans - B)
            errors.append(e)
            results.append([random_selections, trans, e])
        errors = np.array(errors)

        best_estimation_arg = np.argmin(np.median(errors, axis=1))

        errors[errors > self.e_thresh] = np.nan
        nonnan_count = []
        for e in errors:
            nonnan_count.append(np.count_nonzero(~np.isnan(e)))
        nonnan_count = np.asarray(nonnan_count)
        max_arg = np.argmax(nonnan_count)
        inliers = ~np.isnan(errors[max_arg])
        precetage_of_inliers = np.sum(inliers) / inliers.size
        print(f"precetage of inliers in traslation = {precetage_of_inliers}")
        if self.apply_ransac:
            sub_A = A[inliers]
            sub_B = B[inliers]
        else:
            sub_A = A
            sub_B = B
        best_trans = np.linalg.lstsq(sub_A, sub_B, rcond=None)[0]
        e = abs(sub_A @ best_trans - sub_B)
        # plt.hist(e)
        # plt.show()
        a = 0
        return best_trans, np.mean(e), np.std(e)

    def generate_unique_random_numbers(self, N, start, end):
        return random.sample(range(start, end), N)


    def estimate(self, ref, target):
        pass


# # Example usage
# np.random.seed(0)
# src = 100 * np.random.rand(50, 3)
# dst = src + np.random.normal(size=src.shape)
# outliers = np.random.rand(10, 3)
# dst[:10] = outliers
#
# ransac_rigid_transform(src, dst, vis=True)