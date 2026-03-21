// Author: Edward Fang
// Email: edward.fang@berkeley.edu
//
// This code is adapted from
// https://github.com/AtsushiSakai/PythonRobotics/tree/
// master/PathPlanning/FrenetOptimalTrajectory.
// Its author is Atsushi Sakai.
//
// Reference Papers:
// - [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet
// Frame]
// (https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)
// - [Optimal trajectory generation for dynamic street scenarios in a Frenet
// Frame] (https://www.youtube.com/watch?v=Cj6tAQe7UCY)

#ifndef FRENET_OPTIMAL_TRAJECTORY_FRENET_OPTIMAL_TRAJECTORY_H
#define FRENET_OPTIMAL_TRAJECTORY_FRENET_OPTIMAL_TRAJECTORY_H

#include "CubicSpline2D.h"
#include "FrenetPath.h"
#include "Obstacle.h"
#include "py_cpp_struct.h"

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <vector>

using namespace std;
using namespace Eigen;

class FrenetOptimalTrajectory {
public:
  FrenetOptimalTrajectory(FrenetInitialConditions *fot_ic_,
                          FrenetHyperparameters *fot_hp_);
  ~FrenetOptimalTrajectory();
  FrenetPath *getBestPath();
  vector<FrenetPath *> getAllPath();
  void setObstacles();
  void addObstacle(Point_FP first_point, Point_FP second_point,
                   Point_FP third_point, Point_FP fourth_point);
#ifdef SAMPLING_PATH_ANALYSIS
  size_t getSampleCounter() { return _sample_counter; }
  size_t *getSampleLength() { return _sample_length; }
  float *getSampleX() { return _sample_x; }
  float *getSampleY() { return _sample_y; }
#endif

private:
  FrenetInitialConditions *fot_ic;
  FrenetHyperparameters *fot_hp;
  mutex *mu;
  FrenetPath *best_frenet_path;
  CubicSpline2D *csp;
  vector<Obstacle *> obstacles;
  vector<float> x, y; // way points
  vector<FrenetPath *> frenet_paths;
#ifdef SAMPLING_PATH_ANALYSIS
  size_t _sample_counter;
  size_t _sample_length[MAX_SAMPLE_SIZE] = {0};
  float _sample_x[MAX_PATH_LENGTH * MAX_SAMPLE_SIZE] = {0};
  float _sample_y[MAX_PATH_LENGTH * MAX_SAMPLE_SIZE] = {0};
#endif
  void calc_frenet_paths(int start_di_index, int end_di_index,
                         bool multithreaded);
  void threaded_calc_all_frenet_paths();
  void saveBestPathInSameDirection(FrenetPath *path);
};

#endif // FRENET_OPTIMAL_TRAJECTORY_FRENET_OPTIMAL_TRAJECTORY_H
