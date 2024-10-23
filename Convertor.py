import pandas as pd
import numpy as np
import scipy
from scipy.spatial.transform import Rotation as R
n = 5

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

def read_external_data():
    value_to_check = 'OK'

    def check_value(df, column, value):
        new_df = pd.DataFrame(columns=["index", "timestamp", "State", "Q0", "Qx", "Qy", "Qz", "Tx", "Ty", "Tz"])
        for i, row in df.iterrows():
            if row[column] == value:
                new_dict = {"index":i, "timestamp":row["Frame"],"State":row["State"],"Q0":row["Q0"], "Qx":row["Qx"], "Qy":row["Qy"], "Qz":row["Qz"],"Tx":row["Tx"],"Ty":row["Ty"],"Tz":row["Tz"]}
            else:
                new_dict = {"index":i, "timestamp":row["Frame"],"State":row["State"],"Q0":None, "Qx":None, "Qy":None, "Qz":None,"Tx":None,"Ty":None,"Tz":None}
            _df = pd.DataFrame([new_dict])
            new_df = pd.concat([new_df, _df], ignore_index=True)
        return new_df

    df = pd.read_csv(f"{n}.csv")
    result = check_value(df, 'State', value_to_check)
    return result

def read_hololense_data():
    df = pd.DataFrame(columns=["index", "timestamp", "State","Q0","Qx","Qy","Qz","Tx","Ty","Tz"])

    with open(f"{n}.txt", "r") as hl_data:
        lines = hl_data.readlines()
        for i, line in enumerate(lines):
            content = line.split(",")
            data = []

            for d in content[1:]:
                data.append(float(d))
            data_np = np.array(data).reshape((4,4))
            q, t = extract_quaternion_and_translation(data_np)
            new_dict = {"index":i, "timestamp":content[0],"State":"OK","Q0":q[0], "Qx":q[1], "Qy":q[2], "Qz":q[3], "Tx":t[0]*1000, "Ty":t[1]*1000, "Tz":t[2]*1000}
            new_df = pd.DataFrame([new_dict])
            df = pd.concat([df, new_df], ignore_index=True)
    return df

ex = read_external_data()
hl = read_hololense_data()
ex.to_csv(f"ex_data{n}.csv")
hl.to_csv(f"hl_data{n}.csv")

a = 0
