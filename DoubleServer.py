import socket
from Config import Config
import threading
import time
from scipy.spatial.transform import Rotation as R
import numpy as np
import pandas as pd
from scipy.spatial.transform import Slerp
import Calibrator
import Utils

samples = []
ex_data = []
holo_data = []
ex_socket = None
holo_socket = None
is_enough_data = False

def decode_response(response):
    str_list = response
    flt_list = [float(l) for l in str_list]
    timestamp = flt_list[0]
    quaternions = flt_list[4:]
    rotation_mat = Utils.quaternion_to_matrix(quaternions)
    translates = flt_list[1:4]
    transformation_mat = np.zeros((4,4))
    transformation_mat[:4,:4] = rotation_mat
    transformation_mat[:3,3] = translates
    return timestamp, transformation_mat

def calibrate_rotation(samples, apply_ransac=True, vis=True):
    print("Calibrating...")
    Calibrator.calibrate_rotation(samples, apply_ransac=apply_ransac, vis=vis)


def encode_transform(F):
    S = ','.join([str(it) for it in (F.flatten())])
    return S

# def handle_ex_client(conn):
#     with open("ex.txt", 'w+') as f:
#         while True:
#             try:
#                 response = conn.recv(4096).decode()
#                 f.writelines(response)
#                 f.writelines("\n")
#                 ex_timestamp, externalSample = decode_response(response.split(','))
#                 new_ex_data = {"timestamp":ex_timestamp, "mat":externalSample}
#                 ex_data.append(new_ex_data)
#             except:
#                 print(f"Error: respose {response}")

def connect(ip, port, name):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((ip, port))
    server_socket.listen()
    print(f"{name} Server listening on {ip}:{port}")
    conn, addr = server_socket.accept()
    print(f"{name} Connected by {addr}")
    return conn, server_socket

def start_holoserver(ip, port, name):
    conn, server_socket = connect(ip=ip, port=port, name=name)
    response = ""
    while True:
        try:
            response = conn.recv(4096).decode()
            holo_timestamp, holoSample = decode_response(response.split(','))
            new_data = {"timestamp":holo_timestamp, "mat":holoSample}
            # print(f"{name} data recieved")

            holo_data.append(new_data)


        except ConnectionResetError:
            print('Connection reset by peer')
            conn, server_socket = connect(ip=ip, port=port, name=name)
        except:
            print(f"{name} Error: respose {response}")

def start_exserver(ip, port, name):
    conn, server_socket = connect(ip=ip, port=port, name=name)
    response = ""
    while True:
        try:
            response = conn.recv(4096).decode()
            holo_timestamp, holoSample = decode_response(response.split(','))
            new_data = {"timestamp":holo_timestamp, "mat":holoSample}
            # print(f"{name} data recieved")
            ex_data.append(new_data)
        except ConnectionResetError:
            print('Connection reset by peer')
            conn, server_socket = connect(ip=ip, port=port, name=name)
        except:
            print(f"{name} Error: respose {response}")

def start_rec_server(ip='127.0.0.1', port=65430):
    rec_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    rec_socket.bind((ip, port))
    rec_socket.listen()
    print(f"rec Server listening on {ip}:{port}")
    rec_conn, addr = rec_socket.accept()
    print(f"ex Connected by {addr}")
    return rec_conn

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

if __name__ == "__main__":
    enough_thresh = Config.Enough_samples
    threading.Thread(target=start_holoserver, args=("127.0.0.1",65432,"holo")).start()
    threading.Thread(target=start_exserver, args=("127.0.0.1",65431,"extern")).start()

    rec_conn = start_rec_server()

    while True:
        print(f"ex data recieved: {len(ex_data)}, holo data recieved:{len(holo_data)}")
        if (len(ex_data) > 0 and len(holo_data)>0):
            is_enough_data, (ex_index, holo_index) = calc_data_enough(enough_thresh=enough_thresh)
            if is_enough_data:
                samples = find_samples(ex_index, holo_index, enough_thresh=enough_thresh)
                # print(samples)
                rot, rot_err, inliers, deltas = Calibrator.calibrate_rotation(samples, apply_ransac=True, vis=False)
                rotated_samples = Calibrator.rotate_samples(samples, rot)
                trans, trans_err, tras_std = Calibrator.calibrate_translation(rotated_samples)
                F = Calibrator.make_homogeneous(rot.T)
                F[:3,3] = trans
                T = Calibrator.find_T(samples, F)
                FT_trnsforms = encode_transform(F) + '|' + encode_transform(T)
                print("result sent")
                rec_conn.send(FT_trnsforms.encode())

                ex_data = ex_data[len(ex_data)-10:]
                holo_data = holo_data[len(holo_data)-10:]

                is_enough_data = False
        time.sleep(1.0)

