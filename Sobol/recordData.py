import pandas as pd
initial_conditions_floating_point = []
calculated_data_floating_point = []
record_hyperparameters =[]

# class FrenetInitialConditions(Structure):
#     _fields_ = [
#         ("s0", c_float),
#         ("c_speed", c_float),
#         ("c_acceleration", c_float),
#         ("c_d", c_float),
#         ("c_d_d", c_float),
#         ("c_d_dd", c_float),
#         ("target_speed", c_float),
#         ("wx", _c_float_p),
#         ("wy", _c_float_p),
#         ("nw", c_int),
#         ("o_llx", _c_float_p),
#         ("o_lly", _c_float_p),
#         ("o_urx", _c_float_p),
#         ("o_ury", _c_float_p),
#         ("no", c_int)
#     ]
    
def writeInitalConditions(initial_conditions):
    initial_conditions_floating_point.append([
    initial_conditions.s0,
    initial_conditions.c_speed,
    # initial_conditions.c_acceleration,
    initial_conditions.c_d,
    initial_conditions.c_d_d,
    initial_conditions.c_d_dd,
    initial_conditions.target_speed,
    [initial_conditions.wx[i] for i in range(initial_conditions.nw)],
    [initial_conditions.wy[i] for i in range(initial_conditions.nw)],
    initial_conditions.nw,
    [initial_conditions.o_llx[i] for i in range(initial_conditions.no)],
    [initial_conditions.o_lly[i] for i in range(initial_conditions.no)],
    [initial_conditions.o_urx[i] for i in range(initial_conditions.no)],
    [initial_conditions.o_ury[i] for i in range(initial_conditions.no)],
    initial_conditions.no
])

def writeCalculatedData(returnValues, step_number):
    for i in range(returnValues.path_length):
        calculated_data_floating_point.append([
            step_number,
            returnValues.success,
            returnValues.x_path[i],
            returnValues.y_path[i],
            returnValues.speeds[i],
            returnValues.accelerations[i],
            returnValues.ix[i],
            returnValues.iy[i],
            returnValues.iyaw[i],
            returnValues.d[i],
            returnValues.s[i],
            returnValues.speeds_x[i],
            returnValues.speeds_y[i],
            returnValues.costs[11],
            returnValues.yaw[i],
            returnValues.runtime
        ])

def writeHyperparameters(hyperparameters):
    record_hyperparameters.append([
        hyperparameters.max_speed,
        hyperparameters.max_accel,
        hyperparameters.max_curvature,
        hyperparameters.max_road_width_l,
        hyperparameters.max_road_width_r,
        hyperparameters.d_road_w,
        hyperparameters.dt,
        hyperparameters.maxt,
        hyperparameters.mint,
        hyperparameters.d_t_s,
        hyperparameters.n_s_sample,
        hyperparameters.obstacle_clearance,
        hyperparameters.kd,
        hyperparameters.kv,
        hyperparameters.ka,
        hyperparameters.kj,
        hyperparameters.kt,
        hyperparameters.ko,
        hyperparameters.klat,
        hyperparameters.klon,
        hyperparameters.num_threads
    ])

def saveInitialConditions():
    df = pd.DataFrame(initial_conditions_floating_point, columns=[
        's0', 'c_speed', 'c_d', 'c_d_d', 'c_d_dd', 'target_speed', 
        'wx', 'wy', 'nw', 'o_llx', 'o_lly', 'o_urx', 'o_ury', 'no'
    ])
    df.to_csv('FloatingPoint_InitData.csv', index=False, mode='w')

def saveCalculatedData():
    df_calculated = pd.DataFrame(calculated_data_floating_point, columns=[
        'step', 'success', 'x_path', 'y_path', 'speeds', 'accelerations', 'ix', 'iy', 'iyaw', 
        'd', 's', 'speeds_x', 'speeds_y', 'costs', 'yaw', 'runtime'
    ])
    df_calculated.to_csv('FloatingPoint_Calculated.csv', index=False, mode='w')

def saveHyperparameter():
    hp_df = pd.DataFrame(record_hyperparameters, columns=[
        'max_speed', 'max_accel', 'max_curvature', 'max_road_width_l', 'max_road_width_r', 
        'd_road_w', 'dt', 'maxt', 'mint', 'd_t_s', 'n_s_sample', 'obstacle_clearance', 
        'kd', 'kv', 'ka', 'kj', 'kt', 'ko', 'klat', 'klon', 'num_threads'
    ])
    hp_df.to_csv('FloatingPoint_Hyperparameters.csv', index=False, mode='w')