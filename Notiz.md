
1. I tried to convert all variables of interface from python in Branch Make_ValidChecker. However,the runtime is worse. That could because that some variable
   need much computation and it does not worth to use fixed point. TODO: analyse the computation of variable and change only the varibales which need high memory access but low computation
2. The files in polynomials forder is  using fixed point for calculation. But it runtime could be slower. TODO: test the runtime by changing back to float
3. Eigen lib is used for matrix computation. Some variables in CubinSpline1D is defined as double.

Runtime measurment:

1. Configuration:

```python
def fot(show_animation=True,
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
        'obs': [[25,-2,28,2],[29,2,30,3],[29,-1,30,1],[30.5,1,31,4],[31,-1,33,1],
                [32,3,33,4],[35,4,40,6],[48, -5, 52, -6], [53,-4,55,-5],[53,3,55,5],
                [56,-4,58,-5],[56,2,58,5],[59,-4,61,-5],[59,2,61,5],[62,-4,63,-5],
                [62,2,63,5],[65,-4,68,-5],[65,2,68,5],[85, -4, 90, 1], [85, 6, 90, 10]],
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
    obs = np.array(conds['obs'])

    # simulation config
    sim_loop = 200
    area = 40
    total_time = 0
    total_time_c = 0
    time_list = []
    for i in range(sim_loop):
        # run FOT and keep time
        print("Iteration: {}".format(i))
        start_time = time.time()
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

```
