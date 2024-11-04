import numpy as np
import pandas as pd
import matplotlib
from scipy.spatial.transform import Rotation as R
from Calibrator import calibrate_rotation
from Utils import quaternion_rotation_angle_between, quaternion_to_matrix, angle_from_rotation_matrix

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

def find_mean_dt(ex, hl, SHIFT, COEF, fps):
    delta_ts = []
    hl_ts = []
    ex_ts = []
    for i,hlrow in hl.iterrows():
        t_hl = (hlrow["timestamp"] - hl.loc[0,"timestamp"])/(10**6)/(1/fps)
        hl_ts.append(t_hl)
        i_ex = int(hlrow["index"]*COEF - SHIFT)
        try:
            t_ex = (ex.loc[i_ex, "timestamp"] ) # milisecond
            ex_ts.append(t_ex)
        except:
            continue
        delta_t = t_ex - t_hl
        delta_ts.append(delta_t)
        a=0

    print(f"mean: {np.mean(delta_ts)}, std:{np.std(delta_ts)}")
    # plt.hist(hl["timestamp"].diff())
    # plt.plot(delta_ts)
    # plt.show()
    b=0
    return [min(delta_ts), max(delta_ts)], [np.mean(delta_ts) , np.std(delta_ts)]


def find_nearest_ex_index(pivot, timestamps):
    diffs = abs(timestamps - pivot)
    closest = np.min(diffs)
    if closest > 10.0:
        # print(f"couldn't find a match")
        return None
    closest_index = np.argmin(diffs)
    return closest_index

def calc_deltas(points):
    deltas = []
    for i in range(int(len(points)/2)):
        q1 = [points.iloc[i+0]["Q0"], points.iloc[i+0]["Qx"], points.iloc[i+0]["Qy"], points.iloc[i+0]["Qz"]]
        q2 = [points.iloc[i+1]["Q0"], points.iloc[i+1]["Qx"], points.iloc[i+1]["Qy"], points.iloc[i+1]["Qz"]]
        a = quaternion_rotation_angle_between(q1, q2)
        deltas.append(float(a))
    return np.asarray(deltas)




def calc_loss(ex, hl, SHIFT, COEF, fps=13.6, min_dt=-30000, max_dt=+30000):
    hl_random_points = np.random.randint(0, int(len(hl)), 5000)
    clmns = ["dt", "loss","N", "n"]
    df = pd.DataFrame(columns=clmns)
    # [min_dt, max_dt], [mean, std] = find_mean_dt(ex, hl, SHIFT, COEF, fps=fps)
    # dts = np.linspace(mean-100*std, mean+100*std, 6000)
    dts = np.linspace(min_dt, max_dt, 200)
    for dt in dts:
        ex_points_idxs = []
        hl_points_idxs = []
        for idx in hl_random_points:
            ex_t = (hl.loc[idx,"timestamp"] - hl.loc[0, "timestamp"]) / (10 ** 5) + ex.loc[0, "timestamp"] + dt
            ex_closest_index = find_nearest_ex_index(pivot=ex_t, timestamps=ex["timestamps"])
            if not (ex_closest_index is None):
                ex_points_idxs.append(int(ex_closest_index))
                hl_points_idxs.append(idx)
            else:
                continue


        hl_points = hl.iloc[hl_points_idxs]
        hl_angles = calc_deltas(hl_points)


        ex_points = ex.iloc[ex_points_idxs]
        ex_angles = calc_deltas(ex_points)

        nonenans = np.logical_not(np.isnan(ex_angles))
        loss = np.linalg.norm(ex_angles[nonenans] - hl_angles[nonenans])/len(hl_angles[nonenans])
        loss_dict = {"dt":float(dt), "loss":float(loss), "N":len(hl_angles), "n":len(hl_angles[nonenans])}
        print(loss_dict)
        df = df._append(loss_dict, ignore_index=True)
        df.to_csv(f"loss{n}.csv")

    a = 0

def calc_hl_fps(hl):
    intervals = hl["timestamp"].diff()
    nonnan_intervals = intervals[np.logical_not(np.isnan(intervals))]
    mean_interval = np.mean(nonnan_intervals)
    median_interval = np.median(nonnan_intervals)
    std_interval = np.std(nonnan_intervals)
    intervals_cleaned = intervals.where(nonnan_intervals < median_interval*1.1, nonnan_intervals > median_interval*0.9)

    a = 0

def plot_speeds(hl, ex, dt, scores, diff_angles):
    hl_v = pd.DataFrame(columns=["t", "v", "dt"])

    for i in range(1,hl.shape[0]-1):
        dT_hl = np.sqrt((hl.loc[i,"Tx"] - hl.loc[i-1,"Tx"])**2 + \
                    (hl.loc[i,"Ty"] - hl.loc[i-1,"Ty"])**2 + \
                    (hl.loc[i,"Tz"] - hl.loc[i-1,"Tz"])**2)

        h1 = quaternion_to_matrix([hl.loc[i, "Q0"],hl.loc[i, "Qx"],hl.loc[i, "Qy"],hl.loc[i, "Qz"]])
        h2 = quaternion_to_matrix([hl.loc[i-1, "Q0"],hl.loc[i-1, "Qx"],hl.loc[i-1, "Qy"],hl.loc[i-1, "Qz"]])
        dh = h1 @ h2.T
        ah = angle_from_rotation_matrix(dh)

        dt_hl = hl.loc[i,"timestamp"] - hl.loc[i-1,"timestamp"]
        v = ah/dt_hl
        new_hlv = pd.DataFrame.from_records({"t":[hl.loc[i,"timestamp"]- dt], "v":[v], "dt":dt_hl})
        hl_v = pd.concat([hl_v, new_hlv], ignore_index=True)

    ex_v = pd.DataFrame(columns=["t", "v", "dt"])

    for i in range(1, ex.shape[0] - 1):
        dT_ex = np.sqrt((ex.loc[i, "Tx"] - ex.loc[i - 1, "Tx"]) ** 2 + \
             (ex.loc[i, "Ty"] - ex.loc[i - 1, "Ty"]) ** 2 + \
             (ex.loc[i, "Tz"] - ex.loc[i - 1, "Tz"]) ** 2)

        e1 = quaternion_to_matrix([ex.loc[i, "Q0"], ex.loc[i, "Qx"], ex.loc[i, "Qy"], ex.loc[i, "Qz"]])
        e2 = quaternion_to_matrix([ex.loc[i - 1, "Q0"], ex.loc[i - 1, "Qx"], ex.loc[i - 1, "Qy"], ex.loc[i - 1, "Qz"]])
        de = e1 @ e2.T
        ae = angle_from_rotation_matrix(de)



        dt_ex = ex.loc[i, "timestamp"] - ex.loc[i - 1, "timestamp"]
        v = ae / dt_ex
        new_exv = pd.DataFrame.from_records({"t": [ex.loc[i, "timestamp"]], "v": [v], "dt":dt_ex})
        ex_v = pd.concat([ex_v, new_exv], ignore_index=True)

    # plt.figure()
    fig, ax1 = plt.subplots(figsize=(16,8))

    ax1.plot(hl_v["t"], hl_v["v"],color="blue", label="hl")
    ax1.plot(ex_v["t"], ex_v["v"],color="red", label="ex")

    ax2 = ax1.twinx()
    # ax2.plot(scores["timestamp"]-dt, scores["score"], color="yellow")
    dif_a = diff_angles["ref_a"] - diff_angles["tar_a"]
    ax2.plot(diff_angles["timestamp"]-dt, dif_a, color="yellow", label="diff")
    # ax2.set_ylim([-0.1,+0.1])
    fig.legend()
    fig.show()

def create_transformation_matrix(quaternion, translation):
    rotation_matrix = quaternion_to_matrix(quaternion)
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix[:3, :3]
    transformation_matrix[:3, 3] = translation
    return transformation_matrix

def calc_diff_angles(df):
    angles_df = pd.DataFrame()
    for i in range(df.shape[0]-2):
        t = df.loc[i,"timestamp"]
        ref_mat1 = df.loc[i,"ref"]
        ref_mat2 = df.loc[i+1,"ref"]
        ref_diff = ref_mat1@ref_mat2.T
        ref_a = angle_from_rotation_matrix(ref_diff)

        tar_mat1 = df.loc[i,"tar"]
        tar_mat2 = df.loc[i+1,"tar"]
        tar_diff = tar_mat1@tar_mat2.T
        tar_a = angle_from_rotation_matrix(tar_diff)

        new_angle_df = pd.DataFrame.from_records({"ref_a":[ref_a], "tar_a":[tar_a], "timestamp":[t]})
        angles_df = pd.concat([angles_df,new_angle_df], ignore_index=True)
    return angles_df

def find_samples(hl, ex, dt):
    A = []
    B = []
    samples = []
    df = pd.DataFrame()
    for i in range(int((hl.shape[0])*0.3),int((hl.shape[0])*0.7)):
    # for i in range(500,600):

        ex_t = int(hl.loc[i,"timestamp"] - dt)
        ex_closest_index = find_nearest_ex_index(ex_t, ex["timestamp"])
        if (ex_closest_index is None):
            continue
        t_ex = ex.iloc[ex_closest_index]
        if t_ex.isnull().any():
            continue
        t_hl = hl.loc[i]
        # print(f"ex:{t_ex}\nhl:{t_hl}")
        # find_std_in_angles(t_hl, t_ex)
        M_ex = create_transformation_matrix([t_ex["Q0"],t_ex["Qx"],t_ex["Qy"],t_ex["Qz"]], [t_ex['Tx'],t_ex['Ty'],t_ex['Tz']])
        M_hl = create_transformation_matrix([t_hl["Q0"],t_hl["Qx"],t_hl["Qy"],t_hl["Qz"]], [t_hl['Tx'],t_hl['Ty'],t_hl['Tz']])

        samples.append({'ref':M_ex, 'target':M_hl, "ref_i":ex_closest_index, "ref_t":ex_t+dt, "target_i":i, "target_t":hl.loc[i,'timestamp']})
        new_df = pd.DataFrame.from_records({'ref':[M_ex], 'tar':[M_hl],"timestamp":[hl.loc[i,'timestamp']]})
        df = pd.concat([df, new_df], ignore_index=True)

    diff_angles = calc_diff_angles(df)
    return samples, diff_angles


def find_outliers(hl, outliers, deltas, sampling_ration = 50):
    scores = pd.DataFrame({"timestamp": hl["timestamp"], 'score': [0 for i in range(len(hl["timestamp"]))]})
    for index in range(len(deltas)):
        if outliers[index] == True:
            hl_1 = deltas[index].index_1
            hl_2 = deltas[index].index_2

            scores.loc[hl_1, "score"] += 1.0
            scores.loc[hl_2, "score"] += 1.0

    # plt.plot(scores["timestamp"], scores["score"])
    # plt.title("scores")
    # plt.show()
    return scores

def save_aligned_data(ex, hl ,dt):
    df = pd.DataFrame(columns=["t", "hl_Tx", "hl_Ty","hl_Tz", "hl_Q0", "hl_Qx", "hl_Qy", "hl_Qz","ex_Tx", "ex_Ty","ex_Tz", "ex_Q0", "ex_Qx", "ex_Qy", "ex_Qz"])
    for i in range(1,hl.shape[0]-1):
        j = find_nearest_ex_index(hl.loc[i,"timestamp"] - dt,ex["timestamp"])
        if j is None:
            continue
        new_df = pd.DataFrame.from_records({"t":[hl.loc[i,"timestamp"]], "hl_Tx":[hl.loc[i,"Tx"]], "hl_Ty":[hl.loc[i,"Ty"]], "hl_Tz":[hl.loc[i,"Tz"]],"hl_Q0":[hl.loc[i,"Q0"]],"hl_Qx":[hl.loc[i,"Qx"]],"hl_Qy":[hl.loc[i,"Qy"]],"hl_Qz":[hl.loc[i,"Qz"]],
                                                                       "ex_Tx":[ex.loc[j, "Tx"]], "ex_Ty": [ex.loc[j, "Ty"]], "ex_Tz": [ex.loc[j, "Tz"]], "ex_Q0": [ex.loc[j, "Q0"]], "ex_Qx": [ex.loc[j, "Qx"]], "ex_Qy": [ex.loc[j, "Qy"]],"ex_Qz": [ex.loc[j, "Qz"]]})
        df = pd.concat([df, new_df], ignore_index=True)
        if i % 100 == 0:
            df.to_csv("combined_data.csv")
    df.to_csv("combined_data.csv")

n = 5
sampling_ration = 1
if n == 3:
    SHIFT = [149] # for n = 3
    COEF = [1.506533] # for n = 3
    fps = 20
    min_dt = 50000
    max_dt = 100000
    INTERSECTION_THRESH = 30
elif n == 4:
    SHIFT = [600+56-190-120] # for n = 4
    COEF = [0.66] # for n = 4
    fps = 60
    INTERSECTION_THRESH = 100
elif n == 5:
    SHIFT = [-1020] # for n = 5
    COEF = [0.505] # for n = 5
    fps = 60
    min_dt = -20000
    max_dt = +240000
    INTERSECTION_THRESH = 100
elif n == 6:
    SHIFT = [-948] # for n = 6
    COEF = [0.49321608040201] # for n = 6
    fps = 60
    INTERSECTION_THRESH = 100
elif n == 7:
    SHIFT = [-0] # for n = 7
    COEF = [0.505] # for n = 7
    INTERSECTION_THRESH = 100

ex = pd.read_csv(f"ex_data{n}.csv")
ex.loc[:,"timestamp"] *= ((1/fps)*1000)
hl = pd.read_csv(f"hl_data{n}.csv")
hl.loc[:,"timestamp"] *= (1/10000)
# dt = 13348541484098.867 + 1273 + 11450 - 4
# dt = 13348541496817.867
dt = 13348541496822.9
# save_aligned_data(ex, hl ,dt)
def convert_to_right_hand_quaternion(quat):
    r = R.from_quat(quat)
    rot_matrix = r.as_matrix()
    rot_matrix[:, 2] = -rot_matrix[:, 2]  # Negate the z-axis
    new_quat = R.from_matrix(rot_matrix).as_quat()
    return new_quat
# def plot_dirctions(hl, ex, dt):
#     for i in range(int(hl.shape[0]*0.4), int(hl.shape[0]*0.6)):
#         hl_mat = quaternion_to_matrix([hl.loc[i,"Q0"],hl.loc[i,"Qx"],hl.loc[i,"Qy"],hl.loc[i,"Qz"]])
#         ex_t = hl.loc[i,"timestamp"] - dt
#         ex_idx = find_nearest_ex_index(ex_t,ex["timestamp"])
#         hl_mat = quaternion_to_matrix([hl.loc[i,"Q0"],hl.loc[i,"Qx"],hl.loc[i,"Qy"],hl.loc[i,"Qz"]])


# Apply conversion to each quaternion in the DataFrame
hl_copy = hl.copy()
#df = hl.apply(lambda row: convert_to_right_hand_quaternion([row['Q0'], row['Qx'], row['Qy'], row['Qz']]), axis=1)
#hl[['Q0', 'Qx', 'Qy', 'Qz']] = pd.DataFrame(df.tolist(), index=df.index)

samples,diff_angles = find_samples(hl, ex, dt)
rot, rot_err, inliers, deltas = calibrate_rotation(samples,apply_ransac=False, sampling_ration=sampling_ration, vis=True)
# outliers = np.logical_not(inliers)
# scores = find_outliers(hl, outliers, deltas, sampling_ration)
# plot_speeds(hl, ex, dt, scores, diff_angles)

# optimization

# ddt = 200
# ra = np.linspace(dt - ddt, dt + ddt, 40)
# df = pd.DataFrame()

# for i, t in enumerate(ra):
#
#     samples,diff_angles = find_samples(hl, ex, t)
#     m = (diff_angles["ref_a"] - diff_angles["tar_a"]).mean()
#     new_df = pd.DataFrame.from_records({"mean_diff":[m], "t":[t]})
#     df = pd.concat([new_df, df], ignore_index=True)
#
#     df.to_csv("time-based_opt.csv")



