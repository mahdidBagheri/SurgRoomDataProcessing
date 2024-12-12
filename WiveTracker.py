import pandas as pd
import socket
from Config import Config
import threading
import time
from scipy.spatial.transform import Rotation as R
import numpy as np
from scipy.spatial.transform import Slerp
import Calibrator
import Utils


ex_data = []
holo_data = []
def Convert_to_MatrixDF(datalist:list, df:pd.DataFrame, dt):
    for i, r in df.iterrows():
        transformation = Utils.create_transformation_matrix([np.float64(r[4]),np.float64(r[5]),np.float64(r[6]),np.float64(r[7])],[np.float64(r[1]),np.float64(r[2]),np.float64(r[3])])
        datalist.append([r[0]-dt,transformation])
    return datalist

def find_between_timestamps(timestamp, df):
    i_after = df['timestamp'].searchsorted(timestamp)
    return max(i_after,0)

def interpolate(m_before, m_after, w_before):
    before_axis = Utils.axis_from_rotation_matrix(m_before)
    before_angle = Utils.angle_from_rotation_matrix(m_before)

    after_axis = Utils.axis_from_rotation_matrix(m_after)
    after_angle = Utils.angle_from_rotation_matrix(m_after)

    interpolate_axis = (w_before*before_axis + (1-w_before)*after_axis)
    interpolate_axis /= np.linalg.norm(interpolate_axis)
    interpolate_angle = (w_before*before_angle + (1-w_before)*after_angle)/2
    interpolate_translation = w_before*m_before[:3,3] + (1-w_before)*m_after[:3,3]

    new_trans = Utils.create_transformation_matrix_from_axisangle(interpolate_axis,interpolate_angle,interpolate_translation)
    return new_trans


def weighted_transform(transform1, transform2, t):
    """
    Computes the weighted mean of two transformations (translation and rotation).

    :param transform1: First transformation (4x4 numpy array).
    :param transform2: Second transformation (4x4 numpy array).
    :param t: Weight for the second transformation, 0 <= t <= 1.
    :return: Weighted transformation (4x4 numpy array).
    """
    # Extract translations
    translation1 = transform1[:3, 3]
    translation2 = transform2[:3, 3]

    # Weighted average of translations
    interpolated_translation = (1 - t) * translation1 + t * translation2

    # Extract rotations
    rotation1 = R.from_matrix(transform1[:3, :3])
    rotation2 = R.from_matrix(transform2[:3, :3])

    # Perform SLERP for rotations
    key_times = [0, 1]  # Time points for the rotations
    rotations = R.from_quat([rotation1.as_quat(), rotation2.as_quat()])
    slerp = Slerp(key_times, rotations)
    interpolated_rotation = slerp([t])[0]  # Interpolate at time `t`

    # Construct the interpolated transformation matrix
    interpolated_transform = np.eye(4)
    interpolated_transform[:3, :3] = interpolated_rotation.as_matrix()
    interpolated_transform[:3, 3] = interpolated_translation

    return interpolated_transform

def find_samples(ex_index, holo_index, enough_thresh):
    samples = []
    selected_ex = ex_data[ex_index:ex_index+enough_thresh]
    selected_holo = pd.DataFrame.from_records(holo_data[holo_index:holo_index+enough_thresh])
    selected_ex_df = pd.DataFrame.from_records(selected_ex)

    for r, holo_s in selected_holo.iterrows():
        if r == 0:
            continue
        t_holo = holo_s["timestamp"]
        m_holo = holo_s["mat"]
        if np.all(m_holo[:3,:3] == np.eye(3)): continue
        i_after = find_between_timestamps(t_holo, selected_ex_df)
        i_after = min(i_after, len(selected_ex_df)-1)
        i_before = i_after - 1
        if i_before < 0: continue
        print(f"{i_after},{r}")

        t_before = selected_ex_df.loc[i_before,"timestamp"]
        if i_after < selected_ex_df.shape[0] - 2:
            t_after = selected_ex_df.loc[i_after,"timestamp"]
            m_after = selected_ex_df.loc[i_after,"mat"]
            m_before = selected_ex_df.loc[i_before,"mat"]
            w_before = (t_holo - t_before)/(t_after - t_before)
            interpolated_mat = weighted_transform(m_before, m_after, w_before)
            samples.append({"ref":m_holo,"target":interpolated_mat})
    return samples

def calc_data_enough(enough_thresh = 100):
    if ex_data[0]['timestamp'] > holo_data[0]['timestamp']:
        for i in range(0,len(holo_data)):
            if ex_data[0]['timestamp'] < holo_data[i]['timestamp']:
                length_of_intersection = len(ex_data) - i
                if length_of_intersection > enough_thresh:
                    return True, (0, i)
    elif ex_data[0]['timestamp'] < holo_data[0]['timestamp']:
        for i in range(0,len(ex_data)):
            if holo_data[0]['timestamp'] < ex_data[i]['timestamp']:
                length_of_intersection = len(holo_data) - i
                if length_of_intersection > enough_thresh:
                    return True, (i, 0)
    else:
        length_of_intersection = len(holo_data)
        if length_of_intersection > enough_thresh:
            return True, (0,0)
    return False, (0,0)

if __name__ == "__main__":
    errs = []
    for dt in range(-90,-50,1):
        ExTracker = pd.read_csv("Holo.csv")
        HoloTracker = pd.read_csv("ExTracker.csv")
        ex_data = []
        holo_data = []
        ex_data = Convert_to_MatrixDF(ex_data, ExTracker, dt)
        ex_data = pd.DataFrame.from_records(ex_data, columns=["timestamp", "mat"])
        holo_data = Convert_to_MatrixDF(holo_data, HoloTracker, 0)
        holo_data = pd.DataFrame.from_records(holo_data, columns=["timestamp", "mat"])

        enough_thresh = 4000
        # is_enough_data, (ex_index, holo_index) = calc_data_enough(enough_thresh=enough_thresh)
        samples = find_samples(0, 0, enough_thresh=enough_thresh)
        rot, rot_err, inliers, deltas = Calibrator.calibrate_rotation(samples, apply_ransac=True, vis=False)
        trans, trans_err, tras_std = Calibrator.calibrate_translation(samples, rot)
        errs.append([rot_err,np.sum(inliers) / len(inliers),trans_err,dt])
        pd.DataFrame.from_records(errs).to_csv("errs.csv")
    print(errs)
