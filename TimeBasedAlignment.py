import numpy as np
import pandas as pd
import matplotlib

from Calibrator import calibrate_rotation
from Utils import quaternion_rotation_angle_between, quaternion_to_matrix

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

def plot_speeds(hl, ex, dt, scores):
    hl_v = pd.DataFrame(columns=["t", "v", "dt"])

    for i in range(1,hl.shape[0]-1):
        dT_hl = np.sqrt((hl.loc[i,"Tx"] - hl.loc[i-1,"Tx"])**2 + \
                    (hl.loc[i,"Ty"] - hl.loc[i-1,"Ty"])**2 + \
                    (hl.loc[i,"Tz"] - hl.loc[i-1,"Tz"])**2)

        dt_hl = hl.loc[i,"timestamp"] - hl.loc[i-1,"timestamp"]
        v = dT_hl/dt_hl
        new_hlv = pd.DataFrame.from_records({"t":[hl.loc[i,"timestamp"]- dt], "v":[v], "dt":dt_hl})
        hl_v = pd.concat([hl_v, new_hlv], ignore_index=True)

    ex_v = pd.DataFrame(columns=["t", "v", "dt"])

    for i in range(1, ex.shape[0] - 1):
        dT_ex = np.sqrt((ex.loc[i, "Tx"] - ex.loc[i - 1, "Tx"]) ** 2 + \
             (ex.loc[i, "Ty"] - ex.loc[i - 1, "Ty"]) ** 2 + \
             (ex.loc[i, "Tz"] - ex.loc[i - 1, "Tz"]) ** 2)
        dt_ex = ex.loc[i, "timestamp"] - ex.loc[i - 1, "timestamp"]
        v = dT_ex / dt_ex
        new_exv = pd.DataFrame.from_records({"t": [ex.loc[i, "timestamp"]], "v": [v], "dt":dt_ex})
        ex_v = pd.concat([ex_v, new_exv], ignore_index=True)

    plt.figure()
    fig, ax1 = plt.subplots(figsize=(16,8))

    ax1.plot(hl_v["t"], hl_v["v"], label="hl")
    ax1.plot(ex_v["t"], ex_v["v"], label="ex")

    ax2 = ax1.twinx()
    ax2.plot(hl_v["t"], scores[0:(len(hl_v["t"]))], color="yellow", label="outlier scores")

    plt.legend()
    plt.show()

def create_transformation_matrix(quaternion, translation):
    rotation_matrix = quaternion_to_matrix(quaternion)
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix[:3, :3]
    transformation_matrix[:3, 3] = translation
    return transformation_matrix

def find_samples(hl, ex, dt):
    A = []
    B = []
    samples = []
    for i in range(hl.shape[0]-1):

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

        A.append(M_ex)
        B.append(M_hl)
        samples.append({'ref':M_ex, 'target':M_hl, "ref_i":ex_closest_index, "ref_t":ex_t+dt, "target_i":i, "target_t":hl.loc[i,'timestamp']})

    return samples


def find_outliers(outliers, deltas, sampling_ration = 50):
    scores = np.asarray([0.0 for i in range(len(hl))])
    for index in range(len(deltas)):
        if outliers[index] == True:
            hl_1 = deltas[index].index_1
            hl_2 = deltas[index].index_2
            scores[int(hl_1):min(len(hl)-1,int(hl_1+sampling_ration))] += [1.0 for i in range(min(int(sampling_ration),len(hl)-1-int(hl_1+sampling_ration)))]
            scores[int(hl_2):min(len(hl)-1,int(hl_2+sampling_ration))] += [1.0 for i in range(min(int(sampling_ration),len(hl)-1-int(hl_2+sampling_ration)))]

    scores = np.asarray(scores)
    plt.plot(scores)
    plt.show()
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
sampling_ration = 100
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
dt = 13348541484098.867 + 1273 + 11450 - 4
save_aligned_data(ex, hl ,dt)

# samples = find_samples(hl, ex, dt)
# rot, rot_err, inliers, deltas = calibrate_rotation(samples,sampling_ration=sampling_ration, vis=True)
# outliers = np.logical_not(inliers)
# scores = find_outliers(outliers, deltas, sampling_ration)
# plot_speeds(hl, ex, dt, scores)
# plot_speeds(scores)

