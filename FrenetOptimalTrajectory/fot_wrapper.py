import numpy as np
# import py_cpp_struct as pcs
import os
from Sobol import recordData
from ctypes import c_float, c_int, POINTER, Structure, CDLL, byref

try:
    from py_cpp_struct import FrenetInitialConditions, FrenetHyperparameters, \
        FrenetReturnValues
except:
    from FrenetOptimalTrajectory \
        .py_cpp_struct import FrenetInitialConditions, FrenetHyperparameters, \
         FrenetReturnValues
# print(os.getcwd())
try:
    cdll = CDLL("build/libFrenetOptimalTrajectory.so")
except:
    cdll = CDLL("{}/dependencies/frenet_optimal_trajectory_planner/"
                "build/libFrenetOptimalTrajectory.so".format(
                    os.getenv("PYLOT_HOME")))

_c_float_p = POINTER(c_float)

# func / return type declarations for C++ run_fot
_run_fot = cdll.run_fot
_run_fot.argtypes = (
    POINTER(FrenetInitialConditions),
    POINTER(FrenetHyperparameters),
    POINTER(FrenetReturnValues),
)
_run_fot.restype = None

# func / return type declarations for C++ to_frenet_initial_conditions
_to_frenet_initial_conditions = cdll.to_frenet_initial_conditions
_to_frenet_initial_conditions.restype = None
_to_frenet_initial_conditions.argtypes = (c_float, c_float, c_float, c_float,
                                          c_float, c_float, c_float,
                                          _c_float_p, _c_float_p, c_int,
                                          _c_float_p)

RECORD_DATA = False
step_num = 0

def _parse_hyperparameters(hp):
    return FrenetHyperparameters(
        hp["max_speed"], hp["max_accel"], hp["max_curvature"],
        hp["max_road_width_l"], hp["max_road_width_r"], hp["d_road_w"],
        hp["dt"], hp["maxt"], hp["mint"], hp["d_t_s"], hp["n_s_sample"],
        hp["obstacle_clearance"], hp["kd"], hp["kv"], hp["ka"], hp["kj"],
        hp["kt"], hp["ko"], hp["klat"], hp["klon"], hp["num_threads"])


def run_fot(initial_conditions, hyperparameters):
    """ Return the frenet optimal trajectory given initial conditions in
    cartesian space.

    Args:
        initial_conditions (dict): dict containing the following items
            ps (float): previous longitudinal position
            target_speed (float): target speed [m/s]
            pos (np.ndarray([float, float])): initial position in global coord
            vel (np.ndarray([float, float])): initial velocity [m/s]
            wp (np.ndarray([float, float])): list of global waypoints
            obs (np.ndarray([float, float, float, float])): list of obstacles
                as: [lower left x, lower left y, upper right x, upper right y]

        hyperparameters (dict): a dict of optional hyperparameters
            max_speed (float): maximum speed [m/s]
            max_accel (float): maximum acceleration [m/s^2]
            max_curvature (float): maximum curvature [1/m]
            max_road_width_l (float): maximum road width to the left [m]
            max_road_width_r (float): maximum road width to the right [m]
            d_road_w (float): road width sampling discretization [m]
            dt (float): time sampling discretization [s]
            maxt (float): max prediction horizon [s]
            mint (float): min prediction horizon [s]
            d_t_s (float): target speed sampling discretization [m/s]
            n_s_sample (float): sampling number of target speed
            obstacle_clearance (float): obstacle radius [m]
            kd (float): positional deviation cost
            kv (float): velocity cost
            ka (float): acceleration cost
            kj (float): jerk cost
            kt (float): time cost
            ko (float): dist to obstacle cost
            klat (float): lateral cost
            klon (float): longitudinal cost

    Returns:
        result_x (np.ndarray(float)): x positions of fot, if it exists
        result_y (np.ndarray(float)): y positions of fot, if it exists
        speeds (np.ndarray(float)): speeds of fot, if it exists
        ix (np.ndarray(float)): spline x of fot, if it exists
        iy (np.ndarray(float)): spline y of fot, if it exists
        iyaw (np.ndarray(float)): spline yaws of fot, if it exists
        d (np.ndarray(float)): lateral offset of fot, if it exists
        s (np.ndarray(float)): longitudinal offset of fot, if it exists
        speeds_x (np.ndarray(float)): x speeds of fot, if it exists
        speeds_y (np.ndarray(float)): y speeds of fot, if it exists
        params (dict): next frenet coordinates, if they exist
        costs (dict): costs of best frenet path, if it exists
        success (bool): whether a fot was found or not
        runtime (np.ndarray(float)): run time of run_fot in c
        sample_x_data (np.ndarray(np.ndarray(float))): x positions of sample paths
        sample_y_data (np.ndarray(np.ndarray(float))): y positions of samples paths
    """
    # parse initial conditions and convert to frenet coordinates
    fot_initial_conditions, misc = to_frenet_initial_conditions(
        initial_conditions)
    # parse hyper parameters
    fot_hp = _parse_hyperparameters(hyperparameters)

    # initialize return values
    fot_rv = FrenetReturnValues(0)

    if RECORD_DATA:
        recordData.writeInitalConditions(fot_initial_conditions)
        recordData.writeHyperparameters(fot_hp)

    show_sampling_path = os.environ.get("SHOW_SAMPLING_PATH", False)
    # run the planner
    _run_fot(fot_initial_conditions, fot_hp, fot_rv)

    # if fot_rv.success==False:
    #     empty_array = np.array([], dtype=np.float32)
    #     if int(show_sampling_path):
    #         return empty_array, empty_array, empty_array, empty_array, empty_array, \
    #             empty_array, empty_array, empty_array, empty_array, empty_array, \
    #             empty_array, {}, {}, False, {}, empty_array, empty_array
    #     else:
    #         return empty_array, empty_array, empty_array, empty_array, empty_array, \
    #             empty_array, empty_array, empty_array, empty_array, empty_array, \
    #             empty_array, {}, {}, False, {}

    if RECORD_DATA:
        recordData.writeCalculatedData(fot_rv, step_num)

    x_path = np.array([fot_rv.x_path[i] for i in range(fot_rv.path_length)]).astype(np.float32)
    y_path = np.array([fot_rv.y_path[i] for i in range(fot_rv.path_length)]).astype(np.float32)
    speeds = np.array([fot_rv.speeds[i] for i in range(fot_rv.path_length)]).astype(np.float32)
    accelerations = np.array([fot_rv.accelerations[i] for i in range(fot_rv.path_length)]).astype(np.float32)
    ix = np.array([fot_rv.ix[i] for i in range(fot_rv.path_length)]).astype(np.float32)
    iy = np.array([fot_rv.iy[i] for i in range(fot_rv.path_length)]).astype(np.float32)
    iyaw = np.array([fot_rv.iyaw[i] for i in range(fot_rv.path_length)]).astype(np.float32)
    d = np.array([fot_rv.d[i] for i in range(fot_rv.path_length)]).astype(np.float32)
    s = np.array([fot_rv.s[i] for i in range(fot_rv.path_length)]).astype(np.float32)
    speeds_x = np.array([fot_rv.speeds_x[i] for i in range(fot_rv.path_length)]).astype(np.float32)
    speeds_y = np.array([fot_rv.speeds_y[i] for i in range(fot_rv.path_length)]).astype(np.float32)

    if int(show_sampling_path):
        sample_x_data = []
        sample_y_data = []
        sample_length_data = np.array([fot_rv.sample_length[i] for i in range(fot_rv.sample_size)]).astype(np.int32)
        for i in range(fot_rv.sample_size):
            sample_x_data.append([fot_rv.sample_x[i][j] for j in range(sample_length_data[i])])
            sample_y_data.append([fot_rv.sample_y[i][j] for j in range(sample_length_data[i])])
    
    params = {
        "s": fot_rv.params[0],
        "s_d": fot_rv.params[1],
        "d": fot_rv.params[2],
        "d_d": fot_rv.params[3],
        "d_dd": fot_rv.params[4],
    }
    costs = {
        "c_lateral_deviation": fot_rv.costs[0],
        "c_lateral_velocity": fot_rv.costs[1],
        "c_lateral_acceleration": fot_rv.costs[2],
        "c_lateral_jerk": fot_rv.costs[3],
        "c_lateral": fot_rv.costs[4],
        "c_longitudinal_acceleration": fot_rv.costs[5],
        "c_longitudinal_jerk": fot_rv.costs[6],
        "c_time_taken": fot_rv.costs[7],
        "c_end_speed_deviation": fot_rv.costs[8],
        "c_longitudinal": fot_rv.costs[9],
        "c_inv_dist_to_obstacles": fot_rv.costs[10],
        "cf": fot_rv.costs[11],
    }

    success = fot_rv.success

    runtime = fot_rv.runtime

    if int(show_sampling_path):
        return x_path, y_path, speeds, accelerations, ix, iy, iyaw, d, s, \
            speeds_x, speeds_y, params, costs, success, runtime, sample_x_data, sample_y_data
        
    else:
        return x_path, y_path, speeds, accelerations, ix, iy, iyaw, d, s, \
            speeds_x, speeds_y, params, costs, success, runtime

def to_frenet_initial_conditions(initial_conditions):
    """ Convert the cartesian initial conditions into frenet initial conditions.

    Args:
        initial_conditions (dict): dictionary containing
            ps (float): previous longitudinal position
            target_speed (float): target speed [m/s]
            pos (np.ndarray([float, float])): initial position in global coord
            vel (np.ndarray([float, float])): initial velocity [m/s]
            wp (np.ndarray([float, float])): list of global waypoints
            obs (np.ndarray([float, float, float, float])): list of obstacles
                as: [lower left x, lower left y, upper right x, upper right y]
    Returns:
        FrenetInitialConditions, dictionary for debugging
    """
    # parse the initial conditions
    ps = initial_conditions['ps']
    pos = initial_conditions['pos']
    vel = initial_conditions['vel']
    wp = initial_conditions['wp']
    obs = initial_conditions['obs']
    acc = initial_conditions['acc']
    target_speed = initial_conditions['target_speed']
    if obs.shape[0] == 0:
        obs = np.empty((0, 4)).astype(np.float32)
    x = pos[0].item()
    y = pos[1].item()
    vx = vel[0].item()
    vy = vel[1].item()
    wx = wp[:, 0].astype(np.float32)
    wy = wp[:, 1].astype(np.float32)
    o_llx = np.copy(obs[:, 0]).astype(np.float32)
    o_lly = np.copy(obs[:, 1]).astype(np.float32)
    o_ulx = np.copy(obs[:, 2]).astype(np.float32)
    o_uly = np.copy(obs[:, 3]).astype(np.float32)
    o_urx = np.copy(obs[:, 4]).astype(np.float32)
    o_ury = np.copy(obs[:, 5]).astype(np.float32)
    o_lrx = np.copy(obs[:, 6]).astype(np.float32)
    o_lry = np.copy(obs[:, 7]).astype(np.float32)
    forward_speed = np.hypot(vx, vy).item()
    
    wx = np.ascontiguousarray(wx, dtype=np.float32)
    wy = np.ascontiguousarray(wy, dtype=np.float32)

    # construct return array and convert initial conditions
    misc = np.zeros(6).astype(np.float32)
    _to_frenet_initial_conditions(c_float(ps), c_float(x), c_float(y),
                                  c_float(vx), c_float(vy),
                                  c_float(forward_speed),
                                  c_float(acc),
                                  wx.ctypes.data_as(_c_float_p),
                                  wy.ctypes.data_as(_c_float_p),
                                  c_int(len(wx)),
                                  misc.ctypes.data_as(_c_float_p))

    # return the FrenetInitialConditions structure
    return FrenetInitialConditions(
        misc[0],  # c_s
        misc[1],  # c_speed
        misc[2],  # c_acceleration
        misc[3],  # c_d
        misc[4],  # c_d_d
        misc[5],  # c_d_dd
        target_speed,  # target speed
        wx.ctypes.data_as(_c_float_p),  # waypoints x position
        wy.ctypes.data_as(_c_float_p),  # waypoints y position
        len(wx),
        o_llx.ctypes.data_as(_c_float_p),  # obstacles lower left x
        o_lly.ctypes.data_as(_c_float_p),  # obstacles lower left y
        o_ulx.ctypes.data_as(_c_float_p),  # obstacles upper left x
        o_uly.ctypes.data_as(_c_float_p),  # obstacles upper left y
        o_urx.ctypes.data_as(_c_float_p),  # obstacles upper right x
        o_ury.ctypes.data_as(_c_float_p),  # obstacles upper right y
        o_lrx.ctypes.data_as(_c_float_p),  # obstacles lower right x
        o_lry.ctypes.data_as(_c_float_p),  # obstacles lower right y
        len(o_llx),
    ), misc


#############################################################
# For Anytime Planner Implementation
#############################################################
def query_anytime_planner_path(fot_planner, return_rv_object=False):
    # return value should be separate for each path
    fot_rv = FrenetReturnValues(0)
    fot_planner.get_path(fot_rv)

    x_path = np.array([fot_rv.x_path[i] for i in range(fot_rv.path_length)]).astype(np.float32)
    y_path = np.array([fot_rv.y_path[i] for i in range(fot_rv.path_length)]).astype(np.float32)
    speeds = np.array([fot_rv.speeds[i] for i in range(fot_rv.path_length)]).astype(np.float32)
    ix = np.array([fot_rv.ix[i] for i in range(fot_rv.path_length)]).astype(np.float32)
    iy = np.array([fot_rv.iy[i] for i in range(fot_rv.path_length)]).astype(np.float32)
    iyaw = np.array([fot_rv.iyaw[i] for i in range(fot_rv.path_length)]).astype(np.float32)
    d = np.array([fot_rv.d[i] for i in range(fot_rv.path_length)]).astype(np.float32)
    s = np.array([fot_rv.s[i] for i in range(fot_rv.path_length)]).astype(np.float32)
    speeds_x = np.array([fot_rv.speeds_x[i] for i in range(fot_rv.path_length)]).astype(np.float32)
    speeds_y = np.array([fot_rv.speeds_y[i] for i in range(fot_rv.path_length)]).astype(np.float32)
    params = {
        "s": fot_rv.params[0],
        "s_d": fot_rv.params[1],
        "d": fot_rv.params[2],
        "d_d": fot_rv.params[3],
        "d_dd": fot_rv.params[4],
    }
    costs = {
        "c_lateral_deviation": fot_rv.costs[0],
        "c_lateral_velocity": fot_rv.costs[1],
        "c_lateral_acceleration": fot_rv.costs[2],
        "c_lateral_jerk": fot_rv.costs[3],
        "c_lateral": fot_rv.costs[4],
        "c_longitudinal_acceleration": fot_rv.costs[5],
        "c_longitudinal_jerk": fot_rv.costs[6],
        "c_time_taken": fot_rv.costs[7],
        "c_end_speed_deviation": fot_rv.costs[8],
        "c_longitudinal": fot_rv.costs[9],
        "c_inv_dist_to_obstacles": fot_rv.costs[10],
        "cf": fot_rv.costs[11],
    }

    success = fot_rv.success
    if return_rv_object:
        return x_path, y_path, speeds, ix, iy, iyaw, d, s, \
            speeds_x, speeds_y, params, costs, success, fot_rv
    else:
        return x_path, y_path, speeds, ix, iy, iyaw, d, s, \
            speeds_x, speeds_y, params, costs, success
