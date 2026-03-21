#include "FrenetOptimalTrajectory.h"
#include "FrenetPath.h"
#include "py_cpp_struct.h"
#include "CubicSpline2D.h"
#include "utils.h"
#include "tool/recorder.h"
// #include "cordic.h"

#include <stddef.h>
#include <vector>
#include <time.h>

using namespace std;

// C++ wrapper to expose the FrenetOptimalTrajectory class to python
extern "C" {
    // Compute the frenet optimal trajectory given initial conditions
    // in frenet space.
    //
    // Arguments:
    //      fot_ic (FrenetInitialConditions *):
    //          struct ptr containing relevant initial conditions to compute
    //          Frenet Optimal Trajectory
    //      fot_hp (FrenetHyperparameters *):
    //          struct ptr containing relevant hyperparameters to compute
    //          Frenet Optimal Trajectory
    //      x_path, y_path, speeds (float *):
    //          ptr to storage arrays for Frenet Optimal Trajectory
    //      params (float *):
    //          ptr to store initial conditions for debugging
    //
    // Returns:
    //      1 if successful, 0 if failure
    //      Also stores the Frenet Optimal Trajectory into x_path, y_path,
    //      speeds if it exists
    void run_fot(
            FrenetInitialConditions *fot_ic, FrenetHyperparameters *fot_hp,
            FrenetReturnValues *fot_rv
            ) {
        clock_t start, end;
        start = clock();
        FrenetOptimalTrajectory fot = FrenetOptimalTrajectory(fot_ic, fot_hp);
        end = clock();
        float time_taken = ((float)end-start)/CLOCKS_PER_SEC * 1000; // multiply by 1000 to convert to milliseconds
        
        #ifdef USE_RECORDER
            // write recorded data to csv file
            Recorder::getInstance()->writeDataToCSV();
        #endif

        FrenetPath* best_frenet_path = fot.getBestPath();
        if (best_frenet_path && !best_frenet_path->x.empty()){
            fot_rv->success = 1;
            fot_rv->path_length = std::min(best_frenet_path->x.size(), MAX_PATH_LENGTH);
            for (size_t i = 0; i < fot_rv->path_length; i++) {
                fot_rv->x_path[i] = best_frenet_path->x[i];
                fot_rv->y_path[i] = best_frenet_path->y[i];
                fot_rv->speeds[i] = best_frenet_path->s_d[i];
                fot_rv->accelerations[i] = hypot(best_frenet_path->s_dd[i], best_frenet_path->d_dd[i]);
                fot_rv->ix[i] = best_frenet_path->ix[i];
                fot_rv->iy[i] = best_frenet_path->iy[i];
                fot_rv->iyaw[i] = best_frenet_path->iyaw[i];
                fot_rv->d[i] = best_frenet_path->d[i];
                fot_rv->s[i] = best_frenet_path->s[i];
                fot_rv->yaw[i] = best_frenet_path->yaw[i];
                fot_rv->speeds_x[i] = std::cos(best_frenet_path->yaw[i]) *
                    fot_rv->speeds[i];
                fot_rv->speeds_y[i] = std::sin(best_frenet_path->yaw[i]) *
                    fot_rv->speeds[i];
            }


            // store info for debug
            fot_rv->params[0] = best_frenet_path->s[1];
            fot_rv->params[1] = best_frenet_path->s_d[1];
            fot_rv->params[2] = best_frenet_path->d[1];
            fot_rv->params[3] = best_frenet_path->d_d[1];
            fot_rv->params[4] = best_frenet_path->d_dd[1];

            // store costs for logging
            fot_rv->costs[0] = best_frenet_path->c_lateral_deviation;
            fot_rv->costs[1] = best_frenet_path->c_lateral_velocity;
            fot_rv->costs[2] = best_frenet_path->c_lateral_acceleration;
            fot_rv->costs[3] = best_frenet_path->c_lateral_jerk;
            fot_rv->costs[4] = best_frenet_path->c_lateral;
            fot_rv->costs[5] = best_frenet_path->c_longitudinal_acceleration;
            fot_rv->costs[6] = best_frenet_path->c_longitudinal_jerk;
            fot_rv->costs[7] = best_frenet_path->c_time_taken;
            fot_rv->costs[8] = best_frenet_path->c_end_speed_deviation;
            fot_rv->costs[9] = best_frenet_path->c_longitudinal;
            fot_rv->costs[10] = best_frenet_path->c_inv_dist_to_obstacles;
            fot_rv->costs[11] = best_frenet_path->cf;
        }
        fot_rv->runtime = time_taken;

        #ifdef SAMPLING_PATH_ANALYSIS
            //init all sample paths
            std::fill(std::begin(fot_rv->sample_x), std::end(fot_rv->sample_x), 0.0f);
            std::fill(std::begin(fot_rv->sample_y), std::end(fot_rv->sample_y), 0.0f);
            fot_rv->sample_size = fot.getSampleCounter();
            std::copy(fot.getSampleLength(), fot.getSampleLength() + MAX_SAMPLE_SIZE, fot_rv->sample_length);
            std::copy(fot.getSampleX(), fot.getSampleX() + MAX_PATH_LENGTH*MAX_SAMPLE_SIZE, fot_rv->sample_x);
            std::copy(fot.getSampleY(), fot.getSampleY() + MAX_PATH_LENGTH*MAX_SAMPLE_SIZE, fot_rv->sample_y);
        #endif
    }

    // Convert the initial conditions from cartesian space to frenet space
    void to_frenet_initial_conditions(
            float s0, float x, float y, float vx,
            float vy, float forward_speed, float forward_acceleration, float* xp, float* yp, int np,
            float* initial_conditions
            ) {
        vector<float> wx (xp, xp + np); //np=len(wx), see declartion of _to_frenet_initial_conditions() in fot_wrapper.py 
        vector<float> wy (yp, yp + np);
        CubicSpline2D* csp = new CubicSpline2D(wx, wy);

        // get distance from car to spline and projection
        float s = csp->find_s(x, y, s0);
        float distance = norm(csp->calc_x(s) - x, csp->calc_y(s) - y);
        tuple<float, float> bvec ((csp->calc_x(s) - x) / distance,
                (csp->calc_y(s) - y) / distance);

        // normal spline vector
        float x0 = csp->calc_x(s0);
        float y0 = csp->calc_y(s0);
        float x1 = csp->calc_x(s0 + 2);
        float y1 = csp->calc_y(s0 + 2);

        // unit vector orthog. to spline
        tuple<float, float> tvec (y1-y0, -(x1-x0));
        as_unit_vector(tvec);

        // compute tangent / normal car vectors
        tuple<float, float> fvec (vx, vy);
        as_unit_vector(fvec);

        // get initial conditions in frenet frame
        initial_conditions[0] = s; // current longitudinal position s
        initial_conditions[1] = forward_speed; // speed [m/s]
        initial_conditions[2] = forward_acceleration; // acceleration [m/s2]
        // lateral position c_d [m]
        initial_conditions[3] = copysign(distance, dot(tvec, bvec));
        // lateral speed c_d_d [m/s]
        initial_conditions[4] = -forward_speed * dot(tvec, fvec);
        initial_conditions[5] = 0.0; // lateral acceleration c_d_dd [m/s^2]
        // TODO: add lateral acceleration when CARLA 9.7 is patched (IMU)

        delete csp;
    }
}
