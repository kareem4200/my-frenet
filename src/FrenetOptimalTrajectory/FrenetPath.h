#ifndef FRENET_OPTIMAL_TRAJECTORY_FRENETPATH_H
#define FRENET_OPTIMAL_TRAJECTORY_FRENETPATH_H

#include "py_cpp_struct.h"
#include "CubicSpline2D.h"
#include "Obstacle.h"
#include "Car.h"

#include <vector>
#include <tuple>

using namespace std;
using namespace Eigen;

class FrenetPath {
public:
    // Frenet attributes
    vector<float> t;          // time
    vector<float> d;          // lateral offset
    vector<float> d_d;        // lateral speed
    vector<float> d_dd;       // lateral acceleration
    vector<float> d_ddd;      // lateral jerk
    vector<float> s;          // s position along spline
    vector<float> s_d;        // s speed
    vector<float> s_dd;       // s acceleration
    vector<float> s_ddd;      // s jerk

    // Euclidean attributes
    vector<float> x;          // x position
    vector<float> y;          // y position
    vector<float> yaw;        // yaw in radc
    vector<float> ds;         // speed
    vector<float> c;          // curvature
    // vector<float> distanceToReferenceLine; // distance to center line

    // Debug
    vector<float> ix;
    vector<float> iy;
    vector<float> iyaw;

    // Cost attributes
    // lateral costs
    float c_lateral_deviation = 0.0;
    float c_lateral_velocity = 0.0;
    float c_lateral_acceleration = 0.0;
    float c_lateral_jerk = 0.0;
    float c_lateral = 0.0;

    // longitudinal costs
    float c_longitudinal_acceleration = 0.0;
    float c_longitudinal_jerk = 0.0;
    float c_time_taken = 0.0;
    float c_end_speed_deviation = 0.0;
    float c_longitudinal = 0.0;

    // obstacle costs
    float c_inv_dist_to_obstacles = 0.0;

    // final cost
    float cf = 0.0;

    FrenetPath(FrenetHyperparameters *fot_hp_);
    bool to_global_path(CubicSpline2D* csp);
    bool is_valid_path(const vector<Obstacle *> obstacles);
    bool is_collision(const vector<Obstacle *> obstacles);
    float inverse_distance_to_obstacles(
        const vector<Obstacle *> obstacles);

private:
    // Hyperparameters
    FrenetHyperparameters *fot_hp;
};

#endif //FRENET_OPTIMAL_TRAJECTORY_FRENETPATH_H
