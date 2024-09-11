#include <iostream>

#include "FrenetOptimalTrajectory.h"
#include "QuarticPolynomial.h"
#include "QuinticPolynomial.h"
#include "utils.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif

// Compute the frenet optimal trajectory
FrenetOptimalTrajectory::FrenetOptimalTrajectory(
        FrenetInitialConditions *fot_ic_, FrenetHyperparameters *fot_hp_) {
    // parse the waypoints and obstacles
    fot_ic = fot_ic_;
    fot_hp = fot_hp_;
    x.assign(fot_ic->wx, fot_ic->wx + fot_ic->nw);
    y.assign(fot_ic->wy, fot_ic->wy + fot_ic->nw);
    setObstacles();

    // std::cout <<  << '\n';

    // make sure best_frenet_path is initialized
    best_frenet_path = nullptr;

    // exit if not enough waypoints
    if (x.size() <= 2) {
        return;
    }

    // construct spline path
    csp = new CubicSpline2D(x, y);

    // calculate the trajectories
    calc_frenet_paths();

    // select the best path
    double mincost = INFINITY;
    for (FrenetPath* fp : frenet_paths) {
        if (fp->cf <= mincost) {
            mincost = fp->cf;
            best_frenet_path = fp;
        }
    }
}

FrenetOptimalTrajectory::~FrenetOptimalTrajectory() {
    delete csp;
    for (FrenetPath* fp : frenet_paths) {
        delete fp;
    }

    for (Obstacle* ob : obstacles) {
        delete ob;
    }
}

// Return the best path
FrenetPath* FrenetOptimalTrajectory::getBestPath() {
    return best_frenet_path;
}

// Calculate frenet paths
void FrenetOptimalTrajectory::calc_frenet_paths() {
    double t, ti, tv;
    double lateral_deviation, lateral_velocity, lateral_acceleration, lateral_jerk;
    double longitudinal_acceleration, longitudinal_jerk;
    FrenetPath* fp, *tfp;

    double di = -fot_hp->max_road_width_l;
    // std::cout << di << std::endl;
    // generate path to each offset goal
    while (di <= fot_hp->max_road_width_r) {
        ti = fot_hp->mint;
        // lateral motion planning
        while (ti <= fot_hp->maxt) {
            lateral_deviation = 0;
            lateral_velocity = 0;
            lateral_acceleration = 0;
            lateral_jerk = 0;

            fp = new FrenetPath(fot_hp);
            QuinticPolynomial lat_qp = QuinticPolynomial(
                fot_ic->c_d, fot_ic->c_d_d, fot_ic->c_d_dd, di, 0.0, 0.0, ti
            );

            // construct frenet path
            t = 0;
            while (t <= ti) {
                fp->t.push_back(t);
                // std::cout << fp->t.front() << std::endl;
                fp->d.push_back(lat_qp.calc_point(t));
                fp->d_d.push_back(lat_qp.calc_first_derivative(t));
                fp->d_dd.push_back(lat_qp.calc_second_derivative(t));
                fp->d_ddd.push_back(lat_qp.calc_third_derivative(t));
                lateral_deviation += abs(lat_qp.calc_point(t));
                lateral_velocity += abs(lat_qp.calc_first_derivative(t));
                lateral_acceleration += abs(lat_qp.calc_second_derivative(t));
                lateral_jerk += abs(lat_qp.calc_third_derivative(t));
                t += fot_hp->dt;

                

                #ifdef USE_RECORDER
                    Recorder::getInstance()->saveData<double>("FrenetOptimalTrajectory::calc_frenet_paths()::FrenetPath::s", tfp->s.back());
                    Recorder::getInstance()->saveData<double>("FrenetOptimalTrajectory::calc_frenet_paths()::FrenetPath::s_d", tfp->s_d.back());
                    Recorder::getInstance()->saveData<double>("FrenetOptimalTrajectory::calc_frenet_paths()::FrenetPath::s_dd", tfp->s_dd.back());
                    Recorder::getInstance()->saveData<double>("FrenetOptimalTrajectory::calc_frenet_paths()::FrenetPath::s_ddd", tfp->s_ddd.back());
                #endif

            }

            // velocity keeping
            tv = fot_ic->target_speed - fot_hp->d_t_s * fot_hp->n_s_sample; // target velocity gets changed
            while (tv <= fot_ic->target_speed + fot_hp->d_t_s * fot_hp->n_s_sample) {
                longitudinal_acceleration = 0;
                longitudinal_jerk = 0;

                // copy frenet path
                tfp = new FrenetPath(fot_hp);
                tfp->t.assign(fp->t.begin(), fp->t.end());
                tfp->d.assign(fp->d.begin(), fp->d.end());
                tfp->d_d.assign(fp->d_d.begin(), fp->d_d.end());
                tfp->d_dd.assign(fp->d_dd.begin(), fp->d_dd.end());
                tfp->d_ddd.assign(fp->d_ddd.begin(), fp->d_ddd.end());
                QuarticPolynomial lon_qp = QuarticPolynomial(
                    fot_ic->s0, fot_ic->c_speed, 0.0, tv, 0.0, ti
                );

                // longitudinal motion
                for (double tp : tfp->t) {
                    tfp->s.push_back(lon_qp.calc_point(tp));
                    tfp->s_d.push_back(lon_qp.calc_first_derivative(tp));
                    tfp->s_dd.push_back(lon_qp.calc_second_derivative(tp));
                    tfp->s_ddd.push_back(lon_qp.calc_third_derivative(tp));
                    longitudinal_acceleration += abs(lon_qp.calc_second_derivative(tp));
                    longitudinal_jerk += abs(lon_qp.calc_third_derivative(tp));
                }

                // delete if failure or invalid path
                bool success = tfp->to_global_path(csp);
                if (!success || !tfp->is_valid_path(obstacles)) {
                    // deallocate memory and continue
                    delete tfp;
                    tv += fot_hp->d_t_s;
                    continue;
                }

                // lateral costs
                tfp->c_lateral_deviation = lateral_deviation;
                tfp->c_lateral_velocity = lateral_velocity;
                tfp->c_lateral_acceleration = lateral_acceleration;
                tfp->c_lateral_jerk = lateral_jerk;
                tfp->c_lateral = fot_hp->kd * tfp->c_lateral_deviation +
                                 fot_hp->kv * tfp->c_lateral_velocity +
                                 fot_hp->ka * tfp->c_lateral_acceleration +
                                 fot_hp->kj * tfp->c_lateral_jerk;

                // longitudinal costs
                tfp->c_longitudinal_acceleration = longitudinal_acceleration;
                tfp->c_longitudinal_jerk = longitudinal_jerk;
                tfp->c_end_speed_deviation =
                    abs(fot_ic->target_speed - tfp->s_d.back());
                tfp->c_time_taken = ti;
                tfp->c_longitudinal = fot_hp->ka * tfp->c_longitudinal_acceleration +
                                      fot_hp->kj * tfp->c_longitudinal_jerk +
                                      fot_hp->kt * tfp->c_time_taken +
                                      fot_hp->kd * tfp->c_end_speed_deviation;

                // obstacle costs
                tfp->c_inv_dist_to_obstacles =
                    tfp->inverse_distance_to_obstacles(obstacles);

                // final cost
                tfp->cf = fot_hp->klat * tfp->c_lateral +
                          fot_hp->klon * tfp->c_longitudinal +
                          fot_hp->ko * tfp->c_inv_dist_to_obstacles;

                frenet_paths.push_back(tfp);
                tv += fot_hp->d_t_s;
            }
            ti += fot_hp->dt;
            // make sure to deallocate
            delete fp;
        }
        di += fot_hp->d_road_w;
    }
}

void FrenetOptimalTrajectory::setObstacles() {
    // Construct obstacles
    vector<double> llx(fot_ic->o_llx, fot_ic->o_llx + fot_ic->no);
    vector<double> lly(fot_ic->o_lly, fot_ic->o_lly + fot_ic->no);
    vector<double> urx(fot_ic->o_urx, fot_ic->o_urx + fot_ic->no);
    vector<double> ury(fot_ic->o_ury, fot_ic->o_ury + fot_ic->no);

    for (int i = 0; i < fot_ic->no; i++) {
        addObstacle(
            Vector2f(llx[i], lly[i]),
            Vector2f(urx[i], ury[i])
        );
    }
}

void FrenetOptimalTrajectory::addObstacle(Vector2f first_point, Vector2f second_point) {
    obstacles.push_back(new Obstacle(std::move(first_point),
                                     std::move(second_point),
                                     fot_hp->obstacle_clearance));
}

