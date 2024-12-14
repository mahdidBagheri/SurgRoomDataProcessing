import socket

import matplotlib.pyplot as plt

from Config import Config
import threading
import time
from scipy.spatial.transform import Rotation as R
import numpy as np
import pandas as pd
from scipy.spatial.transform import Slerp
import Calibrator
import Utils
from itertools import compress

samples = []
ex_data = []
holo_data = []
ex_socket = None
holo_socket = None
is_enough_data = False

delta_history = []
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
    new_response = ""
    remained_response = ""
    response = ""
    while True:
        try:

            recieved_response = conn.recv(2048).decode()
            recieved_response = remained_response + recieved_response
            if recieved_response == '':
                raise ConnectionResetError

            splitted_responses = recieved_response.split('@')
            if len(splitted_responses[-1].split(",")) < 8:
                remained_response = splitted_responses[-1]
                splitted_responses = splitted_responses[:-1]
            for fragment in splitted_responses:
                if fragment == "":
                    continue
                frag_list = fragment.split(',')
                holo_timestamp, holoSample = decode_response(frag_list)
                new_data = {"timestamp": holo_timestamp, "mat": holoSample}
                holo_data.append(new_data)

        except ConnectionResetError:
            print('Connection reset by peer')
            server_socket.close()
            conn, server_socket = connect(ip=ip, port=port, name=name)
        except Exception as e:
            print(f"{name} Error: respose {remained_response}, e:{e}")
            time.sleep(0.5)

def start_exserver(ip, port, name):
    conn, server_socket = connect(ip=ip, port=port, name=name)
    remained_response = ""
    response = ""
    while True:
        try:
            response = conn.recv(4096).decode()
            if response == '':
                raise ConnectionResetError
            recieved_response = conn.recv(2048).decode()
            recieved_response = remained_response + recieved_response
            if recieved_response == '':
                raise ConnectionResetError
            splitted_responses = recieved_response.split('@')
            if len(splitted_responses[-1].split(",")) < 8:
                remained_response = splitted_responses[-1]
                splitted_responses = splitted_responses[:-1]
            for fragment in splitted_responses:
                if fragment == "":
                    continue
                frag_list = fragment.split(',')
                ex_timestamp, exSample = decode_response(frag_list)
                new_data = {"timestamp": ex_timestamp, "mat": exSample}
                ex_data.append(new_data)
        except ConnectionResetError:
            print('Connection reset by peer')
            server_socket.close()
            conn, server_socket = connect(ip=ip, port=port, name=name)
        except Exception as e:
            print(f"{name} Error: respose {response}, {e}")
            time.sleep(0.5)
def start_rec_server(ip='127.0.0.1', port=65430):
    rec_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    rec_socket.bind((ip, port))
    rec_socket.listen()
    print(f"rec Server listening on {ip}:{port}")
    rec_conn, addr = rec_socket.accept()
    print(f"ex Connected by {addr}")
    return rec_conn, rec_socket

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

def find_samples(ex_index, holo_index, enough_thresh, dt=0):
    samples = []
    selected_ex = ex_data[ex_index:ex_index+enough_thresh]
    selected_holo = pd.DataFrame.from_records(holo_data[holo_index:holo_index+enough_thresh])
    selected_ex_df = pd.DataFrame.from_records(selected_ex)

    for r, holo_s in selected_holo.iterrows():
        if r == 0:
            continue
        t_holo = holo_s["timestamp"] - dt
        m_holo = holo_s["mat"]
        if np.all(m_holo[:3,:3] == np.eye(3)): continue
        i_after = find_between_timestamps(t_holo, selected_ex_df)
        i_after = min(i_after, len(selected_ex_df)-1)
        i_before = i_after - 1
        if i_before < 0: continue
        # print(f"{i_after},{r}")

        t_before = selected_ex_df.loc[i_before,"timestamp"]
        if i_after < selected_ex_df.shape[0] - 2:
            t_after = selected_ex_df.loc[i_after,"timestamp"]
            if t_after - t_before > Config.maximum_timegap:
                continue
            m_after = selected_ex_df.loc[i_after,"mat"]
            m_before = selected_ex_df.loc[i_before,"mat"]
            w_before = (t_holo - t_before)/(t_after - t_before)
            interpolated_mat = weighted_transform(m_before, m_after, w_before)
            samples.append({"ref":m_holo,"target":interpolated_mat})
    return samples

def find_samples_delaytuned(ex_index, holo_index, enough_thresh):
    deltas = []
    dts = range(Config.holodelay[0],Config.holodelay[1],Config.holodelay[2])
    for dt in dts:
        samples = find_samples(ex_index, holo_index, enough_thresh, dt)
        if(len(samples) == 0):
            continue
        delta = 0
        for i in range(0,len(samples),int(len(samples)/Config.delay_sampling_ration)):
            j = len(samples) - i - 1
            qiref = Utils.matrix_to_quaternion(samples[i]["ref"][:3,:3])
            qjref = Utils.matrix_to_quaternion(samples[j]["ref"][:3,:3])
            qitar = Utils.matrix_to_quaternion(samples[i]["target"][:3,:3])
            qjtar = Utils.matrix_to_quaternion(samples[j]["target"][:3,:3])

            aref = Utils.quaternion_rotation_angle_between(qiref,qjref)
            atar = Utils.quaternion_rotation_angle_between(qitar,qjtar)

            delta += abs((aref - atar))
        deltas.append(delta)

    argmin_delta = np.argmin(deltas)
    # plt.plot(deltas)
    # plt.show()
    print(f"calculated delay: {dts[argmin_delta]}")
    samples = find_samples(ex_index, holo_index, enough_thresh, dts[argmin_delta])
    return samples


def connect_to_servers():
    threading.Thread(target=start_holoserver, args=("127.0.0.1", 65432, "holo")).start()
    threading.Thread(target=start_exserver, args=("127.0.0.1", 65431, "extern")).start()

def SendResponse(message):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        client_socket.connect(("127.0.0.1", 65430))
        client_socket.sendall(message.encode())
    except socket.error as e:
        print(f"Error: {e}")
    finally:
        client_socket.close()

def find_n_min_indexes(lst, n):
    # If n exceeds the length of the list, return the original list of indices
    if n >= len(lst):
        return list(range(len(lst)))

    # Create a list of tuples (index, value)
    indexed_lst = list(enumerate(lst))

    # Sort the list based on the values (second element of each tuple)
    indexed_lst.sort(key=lambda x: x[1])

    # Extract the first n minimum values' indices
    min_indexes = [indexed_lst[i][0] for i in range(n)]

    return min_indexes
def find_best_history(rot, deltas, n):
    errors = []
    ref_points = np.array([delta.ref for delta in deltas])
    target_points = np.array([delta.target for delta in deltas])
    for k in range(ref_points.shape[0]):
        e = np.linalg.norm(rot @ ref_points[k].reshape(3,1) - target_points[k].reshape(3,1))
        errors.append(e)
    min_indexes = find_n_min_indexes(errors, n)
    best_deltas = [deltas[i] for i in min_indexes]
    return best_deltas



if __name__ == "__main__":
    enough_thresh = Config.Enough_samples
    connect_to_servers()
    last_rot_error = 1000000
    last_trans_error = 1000000
    while True:
        try:
            print(f"ex data recieved: {len(ex_data)}, holo data recieved:{len(holo_data)}")
            if (len(ex_data) > 0 and len(holo_data)>0):
                is_enough_data, (ex_index, holo_index) = calc_data_enough(enough_thresh=enough_thresh)
                if is_enough_data:
                    print("calibrating ...")
                    # samples = find_samples_(ex_index, holo_index, enough_thresh=enough_thresh, dt=0)
                    samples = find_samples_delaytuned(ex_index, holo_index, enough_thresh=enough_thresh)
                    rot, rot_err, inliers, deltas = Calibrator.calibrate_rotation(samples, apply_ransac=True, vis=False, init_deltas=delta_history)
                    trans, trans_err, tras_std = Calibrator.calibrate_translation(samples, rot)
                    print(f"last rot:{last_rot_error}, rot_err:{rot_err}, last trans:{last_trans_error}, trans_err:{trans_err}")
                    inlier_deltas = [b for a, b in zip(inliers,deltas) if a]
                    delta_history = find_best_history(rot,delta_history + inlier_deltas, n=Config.history_length)

                    # if (last_rot_error > rot_err and last_trans_error > trans_err):
                    if (True):
                        last_rot_error = rot_err
                        last_trans_error = trans_err
                        F = Calibrator.make_homogeneous(rot.T)
                        F[:3, 3] = trans
                        T = Calibrator.find_T(samples, F)
                        FT_trnsforms = encode_transform(F) + '|' + encode_transform(T)
                        print("result sent")
                        SendResponse(FT_trnsforms)
                    ex_data = ex_data[len(ex_data) - Config.retain_data:]
                    holo_data = holo_data[len(holo_data) - Config.retain_data:]
                    is_enough_data = False
        except Exception as e:
            print(f"{e}")
            ex_data = ex_data[len(ex_data) - Config.retain_data:]
            holo_data = holo_data[len(holo_data) - Config.retain_data:]
        time.sleep(1.0)

