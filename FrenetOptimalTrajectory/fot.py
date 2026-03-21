import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patch
import argparse
import os
# import py_cpp_struct
from pathlib import Path

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
parser.add_argument(
    "-dsp",
    "--display_sampling_paths",
    action="store_true",
    help="show sampling paths in animation")
args = parser.parse_args()

os.environ["SHOW_SAMPLING_PATH"] = "1" if args.display_sampling_paths else "0"
from py_cpp_struct import FrenetReturnValues
import py_cpp_struct
import fot_wrapper

# Run fot planner
def fot(show_animation=True,
        show_info=False,
        num_threads=0,
        save_frame=False):
    conds = {
        's0':
        0,
        'target_speed':
        20,
        'wp': [[0, 0], [50, 0], [150, 0]],
        'obs': [[48, -2, 52, 2], [98, -4, 102, 2], [98, 6, 102, 10],
                [128, 2, 132, 6]],
        'pos': [0, 0],
        'vel': [0, 0],
    }  # paste output from debug log

    initial_conditions = {
        'ps': conds['s0'],
        'target_speed': conds['target_speed'],
        'pos': np.array(conds['pos']),
        'vel': np.array(conds['vel']),
        'wp': np.array(conds['wp']),
        'obs': np.array(conds['obs'])
    }

    hyperparameters = {
        "max_speed": 25.0,
        "max_accel": 15.0,
        "max_curvature": 15.0,
        "max_road_width_l": 5.0,
        "max_road_width_r": 5.0,
        "d_road_w": 1,
        "dt": 0.2,
        "maxt": 4.0,
        "mint": 2.0,
        "d_t_s": 0.5,
        "n_s_sample": 2.0,
        "obstacle_clearance": 0.1,
        "kd": 1.0,
        "kv": 0.1,
        "ka": 0.1,
        "kj": 0.1,
        "kt": 0.1,
        "ko": 0.1,
        "klat": 1.0,
        "klon": 1.0,
        "num_threads": num_threads,  # set 0 to avoid using threaded algorithm
        #total path number = ((max_road_width_l+max_road_width_r)/d_road_w + 1)*(2*n_s_sample+1)*(maxt/dt+1)
    }

    # static elements of planner
    wx = initial_conditions['wp'][:, 0]
    wy = initial_conditions['wp'][:, 1]
    obs = np.array(conds['obs'])

    print(os.getpid())

    # simulation config
    sim_loop = 200
    area = 40
    total_time = 0
    total_time_c = 0
    time_list = []

    show_sampling_path = os.environ.get("SHOW_SAMPLING_PATH", False)

    for i in range(sim_loop):
        # run FOT and keep time
        print("Iteration: {}".format(i))
        start_time = time.time()
        if show_sampling_path:
            result_x, result_y, speeds, ix, iy, iyaw, d, s, speeds_x, \
                speeds_y, misc, costs, success, runtime_c, sample_x, sample_y = \
                fot_wrapper.run_fot(initial_conditions, hyperparameters)  
        else:
            result_x, result_y, speeds, ix, iy, iyaw, d, s, speeds_x, \
                speeds_y, misc, costs, success, runtime_c = \
                fot_wrapper.run_fot(initial_conditions, hyperparameters)
                 
        end_time = time.time() - start_time
        # print("Time taken: {} s".format(end_time))
        print("Time take by c module:{} ms".format(runtime_c))
        total_time += end_time
        total_time_c += runtime_c
        time_list.append(runtime_c)

        # reconstruct initial_conditions
        if success:
            initial_conditions['pos'] = np.array([result_x[1], result_y[1]])
            initial_conditions['vel'] = np.array([speeds_x[1], speeds_y[1]])
            initial_conditions['ps'] = misc['s']
            if show_info:
                print(costs)
        else:
            print("Failed unexpectedly")
            break

        # break if near goal
        if np.hypot(result_x[1] - wx[-1], result_y[1] - wy[-1]) <= 3.0:
            print("Goal")
            break

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                "key_release_event",
                lambda event: [exit(0) if event.key == "escape" else None])
            plt.plot(wx, wy)
            if obs.shape[0] == 0:
                obs = np.empty((0, 4))
            ax = plt.gca()
            for o in obs:
                rect = patch.Rectangle((o[0], o[1]), o[2] - o[0], o[3] - o[1])
                ax.add_patch(rect)
            plt.plot(result_x[1:], result_y[1:], "-or")
            plt.plot(result_x[1], result_y[1], "vc")
            plt.xlim(result_x[1] - area, result_x[1] + area)
            plt.ylim(result_y[1] - area, result_y[1] + area)
            if show_sampling_path:
                for path_x, path_y in zip(sample_x, sample_y):
                    plt.plot(path_x, path_y, color='grey', linestyle='--', linewidth=0.5)
            plt.xlabel("X axis")
            plt.ylabel("Y axis")
            plt.title("v[m/s]:" +
                      str(np.linalg.norm(initial_conditions['vel']))[0:4])
            plt.grid(True)
            if save_frame:
                Path("img/frames").mkdir(parents=True, exist_ok=True)
                plt.savefig("img/frames/{}.jpg".format(i))
            plt.pause(0.1)

    print("Finish")

    print("======================= SUMMARY ========================")
    print("Total time for {} iterations taken: {} ms".format(i, total_time_c))
    print("Average time per iteration: {} ms".format(total_time_c / i))
    print("Max time per iteration: {} ms".format(max(time_list)))

    return time_list


if __name__ == '__main__':
    args = parser.parse_args()
    fot(args.display, args.verbose, args.thread, args.save)