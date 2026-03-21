#ifndef FRENETOPTIMALTRAJECTORY_PY_CPP_STRUCT_H
#define FRENETOPTIMALTRAJECTORY_PY_CPP_STRUCT_H
#include <stddef.h>

const size_t MAX_PATH_LENGTH = 100;
const size_t MAX_SAMPLE_SIZE = 100;

struct FrenetInitialConditions {
  float s0;
  float c_speed;
  float c_acceleration;
  float c_d;
  float c_d_d;
  float c_d_dd;
  float target_speed;
  float *wx; // goal position
  float *wy; // goal position
  int nw;
  float *o_llx;
  float *o_lly;
  float *o_ulx;
  float *o_uly;
  float *o_urx;
  float *o_ury;
  float *o_lrx;
  float *o_lry;
  int no;
};

struct FrenetReturnValues {
  int success;
  size_t path_length;
  float x_path[MAX_PATH_LENGTH];
  float y_path[MAX_PATH_LENGTH];
  float speeds[MAX_PATH_LENGTH];
  float accelerations[MAX_PATH_LENGTH];
  float ix[MAX_PATH_LENGTH];
  float iy[MAX_PATH_LENGTH];
  float iyaw[MAX_PATH_LENGTH];
  float d[MAX_PATH_LENGTH];
  float s[MAX_PATH_LENGTH];
  float speeds_x[MAX_PATH_LENGTH];
  float speeds_y[MAX_PATH_LENGTH];
  float params[MAX_PATH_LENGTH];
  float costs[MAX_PATH_LENGTH];
  float yaw[MAX_PATH_LENGTH];
  float runtime;
#ifdef SAMPLING_PATH_ANALYSIS
  float sample_x[MAX_PATH_LENGTH * MAX_SAMPLE_SIZE];
  float sample_y[MAX_PATH_LENGTH * MAX_SAMPLE_SIZE];
  size_t sample_length[MAX_SAMPLE_SIZE];
  size_t sample_size;
#endif
};

struct FrenetHyperparameters {
  float max_speed;
  float max_accel;
  float max_curvature;
  float max_road_width_l;
  float max_road_width_r;
  float d_road_w; // delta_road_width
  float dt;
  float maxt;
  float mint;
  float d_t_s;
  float n_s_sample;
  float obstacle_clearance;
  float kd;
  float kv;
  float ka;
  float kj;
  float kt;
  float ko;
  float klat;
  float klon;
  int num_threads;
};
#endif // FRENETOPTIMALTRAJECTORY_PY_CPP_STRUCT_H
