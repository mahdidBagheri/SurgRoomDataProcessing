import numpy as np
import pandas as pd
import matplotlib

from Utils import quaternion_rotation_angle_between

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
    # plt.plot(delta_ts)
    # plt.show()
    b=0
    return [min(delta_ts), max(delta_ts)], [np.mean(delta_ts) , np.std(delta_ts)]


def find_nearest_ex_index(t_hl, ex):
    items = ex["timestamp"]
    diffs = abs(items - t_hl)
    closest = np.min(diffs)
    if closest > 1.2:
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




def find_course_matches(ex, hl, SHIFT, COEF, fps=13.6):
    clmns = ["dt", "loss","N", "n"]
    df = pd.DataFrame(columns=clmns)
    [min_dt, max_dt], [mean, std] = find_mean_dt(ex, hl, SHIFT, COEF, fps=fps)
    dts = np.linspace(mean-3*std, mean+3*std, 600)
    for dt in dts:
        ex_points_idxs = []
        hl_points_idxs = []
        for i, hlrow in hl.iterrows():
            t_hl = (hlrow["timestamp"] - hl.loc[0, "timestamp"]) / (10 ** 6) / (1 / fps) + dt
            ex_closest_index = find_nearest_ex_index(t_hl, ex)
            if not (ex_closest_index is None):
                ex_points_idxs.append(int(ex_closest_index))
                hl_points_idxs.append(i)
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




n = 3
if n == 3:
    SHIFT = [149] # for n = 3
    COEF = [1.506533] # for n = 3
    INTERSECTION_THRESH = 30
elif n == 4:
    SHIFT = [600+56-190-120] # for n = 4
    COEF = [0.66] # for n = 4
    INTERSECTION_THRESH = 100
elif n == 5:
    SHIFT = [-1020] # for n = 5
    COEF = [0.505] # for n = 5
    INTERSECTION_THRESH = 100
elif n == 6:
    SHIFT = [-948] # for n = 6
    COEF = [0.49321608040201] # for n = 6
    INTERSECTION_THRESH = 100
elif n == 7:
    SHIFT = [-0] # for n = 7
    COEF = [0.505] # for n = 7
    INTERSECTION_THRESH = 100

ex = pd.read_csv(f"ex_data{n}.csv")
hl = pd.read_csv(f"hl_data{n}.csv")




coef = COEF[0]
shift = SHIFT[0]
course_matches = find_course_matches(ex, hl, shift, coef)
# opt_matches = find_opt_matches(ex, hl, course_matches)
