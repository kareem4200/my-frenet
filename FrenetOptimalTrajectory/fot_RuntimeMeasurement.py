import fot_wrapper
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patch
import argparse
from pathlib import Path


# Run fot planner
def fot(show_animation=False,
        show_info=False,
        num_threads=0,
        save_frame=False):
    conds = {
        's0':
        0,
        'target_speed':
        20,
        'wp': [[0, 0], [50, 0], [120, 0]],  #way point
        # 'obs': [[48, -2, 52, 2], [98, -4, 102, 2], [98, 6, 102, 10],
        #         [128, 2, 132, 6]],
        'obs': [[1,3,4,6],[0,-5,2,-3],[3,3,5,6],[3,-5,6,-3],[4,3,6,6],[5,-5,6,-3], [7,3,10,6],[7,-5,10,-3],
                [12,3,14,6],[16,3,22,6],[10,-10,12,-8],[12,-12,15,-8],
                [25,-2,28,2],[29,2,30,3],[29,-1,30,1],[30.5,1,31,4],[31,-1,33,1],
                [32,3,33,4],[35,4,40,6],[48, -5, 52, -6], [53,-4,55,-5],[53,3,55,5],
                [56,-4,58,-5],[56,1,58,5],[59,-4,61,-5],[59,1,61,5],[62,-4,63,-5],
                [62,1,63,5],[65,-4,68,-5],[85, -4, 90, 1], [85, 6, 90, 10]],
        'pos': [0, 0],
        'vel': [0, 0],
    }  # paste output from debug log

    initial_conditions = {
        'ps': conds['s0'],
        'target_speed': conds['target_speed'],
        'pos': np.array(conds['pos']).astype(np.float32),
        'vel': np.array(conds['vel']).astype(np.float32),
        'wp': np.array(conds['wp']).astype(np.float32),
        'obs': np.array(conds['obs']).astype(np.float32)
    }

    hyperparameters = {
        "max_speed": 25.0,
        "max_accel": 15.0,
        "max_curvature": 15.0,
        "max_road_width_l": 6.0,
        "max_road_width_r": 6.0,
        "d_road_w": 0.1,
        "dt": 0.1,
        "maxt": 5,
        "mint": 2,
        "d_t_s": 0.1,
        "n_s_sample": 5.0,
        "obstacle_clearance": 0.1,
        "kd": 1.0,
        "kv": 0.1,
        "ka": 0.1,
        "kj": 0.1,
        "kt": 0.1,
        "ko": 10.0,
        "klat": 1.0,
        "klon": 1.0,
        "num_threads": num_threads,  # set 0 to avoid using threaded algorithm
    }

    # static elements of planner
    wx = initial_conditions['wp'][:, 0]
    wy = initial_conditions['wp'][:, 1]
    obs = np.array(conds['obs']).astype(np.float32)

    # simulation config
    sim_loop = 2000
    area = 40
    # total_time = 0
    # time_list = []
    failed = False
    number_loop = 10
    sum_time_for_measurment=0
    for j in range(1,number_loop+1):
        initial_conditions = {
            'ps': conds['s0'],
            'target_speed': conds['target_speed'],
            'pos': np.array(conds['pos']).astype(np.float32),
            'vel': np.array(conds['vel']).astype(np.float32),
            'wp': np.array(conds['wp']).astype(np.float32),
            'obs': np.array(conds['obs']).astype(np.float32)
        }
        total_time_c=0
        time_list = []
        for i in range(sim_loop):
            # run FOT and keep time
            runtime_c = 0
            start_time = time.time()
            result_x, result_y, speeds, ix, iy, iyaw, d, s, speeds_x, \
            speeds_y, misc, costs, success, runtime_c = \
                fot_wrapper.run_fot(initial_conditions, hyperparameters)
            end_time = time.time() - start_time
            # print("Time taken: {}".format(end_time))
            total_time_c += runtime_c
            time_list.append(runtime_c)

            # reconstruct initial_conditions
            if success:
                initial_conditions['pos'] = np.array([result_x[1], result_y[1]]).astype(np.float32)
                initial_conditions['vel'] = np.array([speeds_x[1], speeds_y[1]]).astype(np.float32)
                initial_conditions['ps'] = misc['s']
                if show_info:
                    print(costs)
            else:
                # print("Failed unexpectedly")
                failed=True
                break

            # break if near goal
            if np.hypot(result_x[1] - wx[-1], result_y[1] - wy[-1]) <= 3.0:
                # print("Goal")
                break
            if show_animation:  # pragma: no cover
                plt.cla()
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect(
                    "key_release_event",
                    lambda event: [exit(0) if event.key == "escape" else None])
                plt.plot(wx, wy)
                if obs.shape[0] == 0:
                    obs = np.empty((0, 4)).astype(np.float32)
                ax = plt.gca()
                for o in obs:
                    rect = patch.Rectangle((o[0], o[1]), o[2] - o[0], o[3] - o[1])
                    ax.add_patch(rect)
                plt.plot(result_x[1:], result_y[1:], "-or")
                plt.plot(result_x[1], result_y[1], "vc")
                plt.xlim(result_x[1] - area, result_x[1] + area)
                plt.ylim(result_y[1] - area, result_y[1] + area)
                plt.xlabel("X axis")
                plt.ylabel("Y axis")
                plt.title("v[m/s]:" +
                        str(np.linalg.norm(initial_conditions['vel']))[0:4])
                plt.grid(True)
                if save_frame:
                    Path("img/frames").mkdir(parents=True, exist_ok=True)
                    plt.savefig("img/frames/{}.jpg".format(i))
                plt.pause(0.1)
        # print("simulation number {}".format(j))
        print("simulation {} run {} cycle, takes average {} ms/cycle".format(j,i,total_time_c/i))
        sum_time_for_measurment += total_time_c/i
    print("Total time for {} Test taken average: {} ms/cycle".format(j, sum_time_for_measurment/j))
    # print("Total time for {} iterations taken: {}".format(i, total_time))
    return time_list


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-d",
        "--display",
        action="store_true",
        help="show animation, ensure you have X11 forwarding server open")
    parser.add_argument("-v",
                        "--verbose",
                        action="store_true",
                        help="verbose mode, show all state info")
    parser.add_argument("-s",
                        "--save",
                        action="store_true",
                        help="save each frame of simulation")
    parser.add_argument("-t",
                        "--thread",
                        type=int,
                        default=0,
                        help="set number of threads to run with")
    args = parser.parse_args()

    # run planner with args passed in
    fot(args.display, args.verbose, args.thread, args.save)
