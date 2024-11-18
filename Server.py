import socket
import csv
import os
import glob
import threading

import numpy as np

# Set the root path manually
import Calibrator
import Utils

root_path = "F:\work\ARassis\\tracking\calibration\CalibrationTest\Assets\Data"

samples = []
def get_last_row_of_last_csv():
    # Get list of all Data[n].csv files in the root_path folder
    list_of_files = glob.glob(os.path.join(root_path, 'Data*.csv'))
    if not list_of_files:
        return "No CSV files found."

    # Find the latest file
    latest_file = max(list_of_files, key=os.path.getctime)

    # Read the last row of the latest CSV file
    with open(latest_file, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        rows = list(csvreader)
        if rows:
            return ','.join(rows[-1])
        else:
            return None

def decode_response(response):
    str_list = response
    flt_list = [float(l) for l in str_list]
    quaternions = flt_list[3:]
    rotation_mat = Utils.quaternion_to_matrix(quaternions)
    translates = flt_list[:3]
    transformation_mat = np.zeros((4,4))
    transformation_mat[:4,:4] = rotation_mat
    transformation_mat[:3,3] = translates
    return transformation_mat


def calibrate_rotation(samples, apply_ransac=True, sampling_ration=1, vis=True):
    print("Calibrating...")
    Calibrator.calibrate_rotation(samples, apply_ransac=True, sampling_ration=1, vis=False)


def encode_transform(F):
    S = ','.join([str(it) for it in (F.flatten())])
    return S



def handle_client(conn):
    samples = []
    while True:
        response = conn.recv(1024).decode()
        if response == "calibrate":
            samples = samples[5:]
            rot, rot_err, inliers, deltas = Calibrator.calibrate_rotation(samples, apply_ransac=True, sampling_ration=1, vis=False)
            rotated_samples = Calibrator.rotate_samples(samples, rot)
            trans, trans_err, tras_std = Calibrator.calibrate_translation(rotated_samples)
            F = Calibrator.make_homogeneous(rot.T)
            F[:3,3] = trans
            T = Calibrator.find_T(samples, F)
            FT_trnsforms = encode_transform(F) + '|' + encode_transform(T)
            conn.sendall(FT_trnsforms.encode())
        else:
            # externalSample = decode_response(get_last_row_of_last_csv())
            holoSample = decode_response(response.split(',')[7:])
            externalSample = decode_response(response.split(',')[:7])
            print(holoSample)
            print(externalSample)
            print("Received")
            samples.append({'ref': externalSample, 'target': holoSample})


def start_server(ip='127.0.0.1', port=65432):
    global server_socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((ip, port))
    server_socket.listen()
    print(f"Server listening on {ip}:{port}")

    conn, addr = server_socket.accept()
    print(f"Connected by {addr}")

    client_thread = threading.Thread(target=handle_client, args=(conn,))
    client_thread.start()


def stop_server():
    server_socket.close()
    print("Server stopped.")


if __name__ == "__main__":
    server_thread = threading.Thread(target=start_server)
    server_thread.start()

    # Listening for 'c' key press to stop the server and run calibration
    while True:
        user_input = input()
        if user_input.lower() == 'c':
            stop_server()
            calibrate_rotation(samples, apply_ransac=True, sampling_ration=1, vis=True)
            break


