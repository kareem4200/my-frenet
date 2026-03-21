#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <time.h>

#include "FrenetOptimalTrajectory.h"
#include "QuarticPolynomial.h"
#include "QuinticPolynomial.h"
#include "utils.h"
#ifdef USE_RECORDER
#include "tool/recorder.h"
#endif

using namespace std;

// Compute the frenet optimal trajectory
FrenetOptimalTrajectory::FrenetOptimalTrajectory(
    FrenetInitialConditions *fot_ic_, FrenetHyperparameters *fot_hp_) {
#ifdef SAMPLING_PATH_ANALYSIS
  _sample_counter = 0;
#endif

  auto start = chrono::high_resolution_clock::now();
  // parse the waypoints and obstacles
  fot_ic = fot_ic_;
  fot_hp = fot_hp_;
  mu = new mutex();

  x.assign(fot_ic->wx, fot_ic->wx + fot_ic->nw);
  y.assign(fot_ic->wy, fot_ic->wy + fot_ic->nw);
  setObstacles();

  // make sure best_frenet_path is initialized
  best_frenet_path = nullptr;

  // exit if not enough waypoints
  if (x.size() < 2) {
    return;
  }

  // construct spline path
  csp = new CubicSpline2D(x, y);

  // calculate the trajectories
  if (fot_hp->num_threads == 0) {
    // calculate how to split computation across threads

    int total_di_iter =
        static_cast<int>((fot_hp->max_road_width_l + fot_hp->max_road_width_r) /
                         fot_hp->d_road_w) +
        1; // account for the last index

    calc_frenet_paths(0, total_di_iter, false);

  } else { // if threading
    threaded_calc_all_frenet_paths();
  }

  // select the best path
  float mincost = INFINITY;
  for (FrenetPath *fp : frenet_paths) {
    if (fp->cf <= mincost) {
      mincost = fp->cf;
      best_frenet_path = fp;
    }
  }
  auto end = chrono::high_resolution_clock::now();
  // float run_time =
  //     chrono::duration_cast<chrono::nanoseconds>(end - start).count();
  // run_time *= 1e-6;
  // cout << "Planning runtime " << run_time << "\n";
}

FrenetOptimalTrajectory::~FrenetOptimalTrajectory() {
  delete mu;
  delete csp;
  for (FrenetPath *fp : frenet_paths) {
    delete fp;
  }

  for (Obstacle *ob : obstacles) {
    delete ob;
  }
}

// Return the best path
FrenetPath *FrenetOptimalTrajectory::getBestPath() { return best_frenet_path; }

/*
 * Spawn threads to calculate frenet paths
 * Called when multithreading.
 */
void FrenetOptimalTrajectory::threaded_calc_all_frenet_paths() {
  vector<thread> threads;

  // calculate how to split computation across threads
  int num_di_iter = static_cast<int>(
      (fot_hp->max_road_width_l + fot_hp->max_road_width_r) / fot_hp->d_road_w);
  num_di_iter = num_di_iter + 1; // account for the last index

  int iter_di_index_range = static_cast<int>(num_di_iter / fot_hp->num_threads);

  for (int i = 0; i < fot_hp->num_threads; i++) {
    if (i != fot_hp->num_threads - 1) {
      threads.push_back(thread(&FrenetOptimalTrajectory::calc_frenet_paths,
                               this, i * iter_di_index_range,
                               (i + 1) * iter_di_index_range, true));
    } else { // account for last thread edge case
      threads.push_back(thread(&FrenetOptimalTrajectory::calc_frenet_paths,
                               this, i * iter_di_index_range, num_di_iter,
                               true));
    }
  }

  // wait for all threads to finish computation
  for (auto &t : threads) {
    t.join();
  }
}

/*
 * Calculate frenet paths
 * If running we are multithreading,
 * We parallelize on the outer loop, in terms of di
 * Iterates over possible values of di, from start index to end index
 * (exclusive). Then, computes the actual di value for path planning.
 * Mutex is only enabled when we are multithreading.
 */
void FrenetOptimalTrajectory::calc_frenet_paths(int start_di_index,
                                                int end_di_index,
                                                bool multithreaded) {
  float t, ti, tv;
  float lateral_deviation, lateral_velocity, lateral_acceleration, lateral_jerk;
  float longitudinal_acceleration, longitudinal_jerk;
  FrenetPath *fp, *tfp;
  int num_paths = 0;
  int num_viable_paths = 0;

  // float valid_path_time = 0;

  // initialize di, with start_di_index
  float di = -fot_hp->max_road_width_l + start_di_index * fot_hp->d_road_w;

  // generate path to each offset goal
  // note di goes up to but not including end_di_index*fot_hp->d_road_w
  while ((di < -fot_hp->max_road_width_l + end_di_index * fot_hp->d_road_w) &&
         (di <= fot_hp->max_road_width_r)) {
    ti = fot_hp->mint;
    // lateral motion planning

#ifdef SAMPLING_PATH_ANALYSIS
    FrenetPath *best_path_in_one_direction = new FrenetPath(fot_hp);
    float min_cost = INFINITY;
#endif

    while (ti <= fot_hp->maxt) {
      lateral_deviation = 0;
      lateral_velocity = 0;
      lateral_acceleration = 0;
      lateral_jerk = 0;

      fp = new FrenetPath(fot_hp);

      QuinticPolynomial lat_qp = QuinticPolynomial(
          fot_ic->c_d, fot_ic->c_d_d, fot_ic->c_d_dd, di, 0.0, 0.0, ti);

      // construct frenet path
      t = 0;
      while (t <= ti) {
        fp->t.push_back(t);
        fp->d.push_back(lat_qp.calc_point(t));
        fp->d_d.push_back(lat_qp.calc_first_derivative(t));
        fp->d_dd.push_back(lat_qp.calc_second_derivative(t));
        fp->d_ddd.push_back(lat_qp.calc_third_derivative(t));
        lateral_deviation += abs(fp->d.back()); //TODO: use calculated result directly fp->d.back()
        lateral_velocity += abs(fp->d_d.back());
        lateral_acceleration += abs(fp->d_dd.back());
        lateral_jerk += abs(fp->d_ddd.back());
        t += fot_hp->dt;
        // #ifdef USE_RECORDER
        //     Recorder::getInstance()->saveData<float>("calc_frenet_paths::t",
        //     fp->t.back());
        //     Recorder::getInstance()->saveData<float>("calc_frenet_paths::d",
        //     fp->d.back());
        //     Recorder::getInstance()->saveData<float>("calc_frenet_paths::d_d",
        //     fp->d_d.back());
        //     Recorder::getInstance()->saveData<float>("calc_frenet_paths::d_dd",
        //     fp->d_dd.back());
        //     Recorder::getInstance()->saveData<float>("calc_frenet_paths::d_ddd",
        //     fp->d_ddd.back());
        // #endif
      }

      // velocity keeping
      tv = fot_ic->target_speed - fot_hp->d_t_s * fot_hp->n_s_sample;
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

        // std::cout << tv << std::endl;

        QuarticPolynomial lon_qp =
            QuarticPolynomial(fot_ic->s0, fot_ic->c_speed, 0.0, tv, 0.0, ti);

        // longitudinal motion
        for (float tp : tfp->t) {
          tfp->s.push_back(lon_qp.calc_point(tp));
          tfp->s_d.push_back(lon_qp.calc_first_derivative(tp));
          tfp->s_dd.push_back(lon_qp.calc_second_derivative(tp));
          tfp->s_ddd.push_back(lon_qp.calc_third_derivative(tp));
          longitudinal_acceleration += abs(lon_qp.calc_second_derivative(tp));
          longitudinal_jerk += abs(lon_qp.calc_third_derivative(tp));
          // #ifdef USE_RECORDER
          //     Recorder::getInstance()->saveData<float>("calc_frenet_paths::s",
          //     tfp->s.back());
          //     Recorder::getInstance()->saveData<float>("calc_frenet_paths::s_d",
          //     tfp->s_d.back());
          //     Recorder::getInstance()->saveData<float>("calc_frenet_paths::s_dd",
          //     tfp->s_dd.back());
          //     Recorder::getInstance()->saveData<float>("calc_frenet_paths::s_ddd",
          //     tfp->s_ddd.back());
          // #endif
        }
        num_paths++;
        // delete if failure or invalid path
        bool success = tfp->to_global_path(csp);

        num_viable_paths++;
        if (!success) {
          // deallocate memory and continue
          delete tfp;
          tv += fot_hp->d_t_s;
          continue;
        }

        bool valid_path = tfp->is_valid_path(obstacles);
#ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("sampled_velocity", tv);
        Recorder::getInstance()->saveData<float>("valid_path",
                                                 static_cast<int>(valid_path));
#endif
        // std::cout << "tv: " << tv << " valid_path: " << valid_path <<
        // std::endl;
        if (!valid_path) {
          delete tfp;
          tv += fot_hp->d_t_s;
          continue;
        }

        // d distance cost
        // double d_cost = 0.0;
        // for (size_t i = 0; i < tfp->d.size(); ++i) {
        //   d_cost += std::pow((0.25 * (0 - tfp->d[i])), 2);
        // }
        // d_cost += std::pow((20 * (0 - tfp->d.back())), 2);

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
        // tfp->c_inv_dist_to_obstacles =
        //     tfp->inverse_distance_to_obstacles(obstacles);

        // final cost
        // tfp->cf = fot_hp->klat * tfp->c_lateral +
        //           fot_hp->klon * tfp->c_longitudinal +
        //           fot_hp->ko * tfp->c_inv_dist_to_obstacles;

        // when lon. jerk cost is enabled, it considers waiting a jerky motion
        // and crashes when it's not negative accelerations are generated and
        // too much lateral jerk
        tfp->cf = fot_hp->klat * tfp->c_lateral +
                  fot_hp->klon * tfp->c_longitudinal;
#ifdef SAMPLING_PATH_ANALYSIS
        if (tfp->cf < min_cost) {
          min_cost = tfp->cf;
          *best_path_in_one_direction = *tfp;
        }
#endif
        frenet_paths.push_back(tfp);
        tv += fot_hp->d_t_s;
      }
      ti += fot_hp->dt;
      // make sure to deallocate
      delete fp;
    }
#ifdef SAMPLING_PATH_ANALYSIS
    if (best_path_in_one_direction->x.size() > 0) {
      saveBestPathInSameDirection(best_path_in_one_direction);
      _sample_length[_sample_counter] = (best_path_in_one_direction->x).size();
      _sample_counter++;
      delete best_path_in_one_direction;
    }
#endif
    di += fot_hp->d_road_w;
  }
}

#ifdef SAMPLING_PATH_ANALYSIS
void FrenetOptimalTrajectory::saveBestPathInSameDirection(FrenetPath *path) {
  for (size_t i = 0; i < path->x.size(); i++) {
    _sample_x[_sample_counter * MAX_PATH_LENGTH + i] = path->x[i];
    _sample_y[_sample_counter * MAX_PATH_LENGTH + i] = path->y[i];
  }
}
#endif

void FrenetOptimalTrajectory::setObstacles() {
  // Construct obstacles
  vector<float> llx(fot_ic->o_llx, fot_ic->o_llx + fot_ic->no);
  vector<float> lly(fot_ic->o_lly, fot_ic->o_lly + fot_ic->no);
  vector<float> ulx(fot_ic->o_ulx, fot_ic->o_ulx + fot_ic->no);
  vector<float> uly(fot_ic->o_uly, fot_ic->o_uly + fot_ic->no);
  vector<float> urx(fot_ic->o_urx, fot_ic->o_urx + fot_ic->no);
  vector<float> ury(fot_ic->o_ury, fot_ic->o_ury + fot_ic->no);
  vector<float> lrx(fot_ic->o_lrx, fot_ic->o_lrx + fot_ic->no);
  vector<float> lry(fot_ic->o_lry, fot_ic->o_lry + fot_ic->no);

  for (int i = 0; i < fot_ic->no; i++) {
    addObstacle(Point_FP(llx[i], lly[i]), Point_FP(ulx[i], uly[i]),
                Point_FP(urx[i], ury[i]), Point_FP(lrx[i], lry[i]));
  }
}

void FrenetOptimalTrajectory::addObstacle(Point_FP first_point,
                                          Point_FP second_point,
                                          Point_FP third_point,
                                          Point_FP fourth_point) {
  obstacles.push_back(
      new Obstacle(std::move(first_point), std::move(second_point),
                   std::move(third_point), std::move(fourth_point)));
}

vector<FrenetPath *> FrenetOptimalTrajectory::getAllPath() {
  return frenet_paths;
}