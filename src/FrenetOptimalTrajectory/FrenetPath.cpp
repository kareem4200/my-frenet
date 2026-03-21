#include "FrenetPath.h"
#include "utils.h"
#ifdef USE_RECORDER
#include "tool/recorder.h"
#endif

#include <algorithm>

const float COLLISION_CHECK_THRESHOLD = 6; // don't check unless within 6m

FrenetPath::FrenetPath(FrenetHyperparameters *fot_hp_) { fot_hp = fot_hp_; }

// Convert the frenet path to global path in terms of x, y, yaw, velocity
bool FrenetPath::to_global_path(CubicSpline2D *csp) {
  float ix_, iy_, iyaw_, di, fx, fy, dx, dy;
  // calc global positions
  for (size_t i = 0; i < s.size(); i++) {
    float s_i = static_cast<float>(s[i]);
    ix_ = csp->calc_x(s_i);
    iy_ = csp->calc_y(s_i);
    if (isnan(ix_) || isnan(iy_))
      break;

        ix.push_back(ix_);
        iy.push_back(iy_);
        iyaw_ = csp->calc_yaw(s_i);
        iyaw.push_back(iyaw_);
        di = d[i];
        fx = ix_ + di * cos(iyaw_ + M_PI_2);
        fy = iy_ + di * sin(iyaw_ + M_PI_2);
        x.push_back(fx);
        y.push_back(fy);
        // distanceToReferenceLine.push_back(
        //     hypot(fx - ix_, fy - iy_));
        
        // #ifdef USE_RECORDER
        //     Recorder::getInstance()->saveData<float>("FrenetPath::to_global_path::i", static_cast<float>(i));
        //     Recorder::getInstance()->saveData<float>("FrenetPath::to_global_path::d[i]", static_cast<float>(d[i]));
        //     Recorder::getInstance()->saveData<float>("FrenetPath::to_global_path::s[i]", static_cast<float>(s[i]));
        //     Recorder::getInstance()->saveData<float>("FrenetPath::to_global_path::ix", static_cast<float>(ix.back()));
        //     Recorder::getInstance()->saveData<float>("FrenetPath::to_global_path::iy", static_cast<float>(iy.back()));
        //     Recorder::getInstance()->saveData<float>("FrenetPath::to_global_path::x", static_cast<float>(x.back()));
        //     Recorder::getInstance()->saveData<float>("FrenetPath::to_global_path::y", static_cast<float>(y.back()));
        //     Recorder::getInstance()->saveData<float>("FrenetPath::to_global_path::iyaw", static_cast<float>(iyaw.back()));
        //     Recorder::getInstance()->saveData<float>("FrenetPath::to_global_path::distanceToReferenceLine", static_cast<float>(distanceToReferenceLine.back()));
        // #endif
    }

  // not enough points to construct a valid path
  if (x.size() <= 1) {
    return false;
  }

    // calc yaw and ds
    for (size_t i = 0; i < x.size() - 1; i++) {
        dx = static_cast<float>(x[i+1] - x[i]);
        dy = static_cast<float>(y[i+1] - y[i]);
        yaw.push_back(atan2(dy, dx));
        ds.push_back(hypot(dx, dy));
        // #ifdef USE_RECORDER
        //     Recorder::getInstance()->saveData<float>("FrenetPath::to_global_path::yaw", yaw.back());
        //     Recorder::getInstance()->saveData<float>("FrenetPath::to_global_path::ds", ds.back());
        //     // Recorder::getInstance()->saveData<float>("FrenetPath::to_global_path()::FrenetPath::dx", dx);
        //     // Recorder::getInstance()->saveData<float>("FrenetPath::to_global_path()::FrenetPath::dy", dy);
        // #endif
    }
    yaw.push_back(yaw.back());
    ds.push_back(ds.back());

    // calc curvature
    for (size_t i = 0; i < yaw.size() - 1; i++) {
        float dyaw = static_cast<float>(yaw[i+1] - yaw[i]);
        if (dyaw > M_PI_2) {
            dyaw -= M_PI;
        } else if (dyaw < -M_PI_2) {
            dyaw += M_PI;
        }
        c.push_back(dyaw / ds[i]);
        // #ifdef USE_RECORDER
        //     Recorder::getInstance()->saveData<float>("FrenetPath::to_global_path::FrenetPath::c", c.back());
        //     // Recorder::getInstance()->saveData<float>("FrenetPath::to_global_path()::FrenetPath::dyaw", dyaw);
        // #endif
    }

  return true;
}

// Validate the calculated frenet paths against threshold speed, acceleration,
// curvature and collision checks
bool FrenetPath::is_valid_path(const vector<Obstacle *> obstacles) {
    // take the small one max_road_width_l - VEHICLE_WIDTH/2.0 or max_road_width_r - VEHICLE_WIDTH/2.0
    //float min_road_width = std::min(fot_hp->max_road_width_l , fot_hp->max_road_width_r )-VEHICLE_WIDTH/2.0;
    if (any_of(s_d.begin(), s_d.end(),
            [this](auto i){return abs(i) > fot_hp->max_speed;})) {
        return false;
    }
    // max accel check
    else if (any_of(s_dd.begin(), s_dd.end(),
            [this](auto i){return abs(i) > fot_hp->max_accel;})) {
        return false;
    }
    // max curvature check
    else if (any_of(c.begin(), c.end(),
            [this](auto i){return abs(i) > fot_hp->max_curvature;})) {
        return false;
    }
    //out of boundary check
    // else if (any_of(distanceToReferenceLine.begin(),
    //         distanceToReferenceLine.end(),
    //         [&](auto i){return abs(i) > min_road_width;})) {
    //     return false;
    // }
    // collision check
    // else if (is_collision(obstacles)) {
    //     return false;
    // }
    else {
        return true;
    }
}

// check path for collision with obstacles
bool FrenetPath::is_collision(const vector<Obstacle *> obstacles) {
  // no obstacles
  if (obstacles.empty()) {
    return false;
  }

  Pose pose;
  Car car = Car();
  Vector2f p1, p2;
  Rectangle car_outline;
  // iterate over all obstacles
  for (auto obstacle : obstacles) {
    float llx = obstacle->bbox.points[0].x;
    float lly = obstacle->bbox.points[0].y;
    float ulx = obstacle->bbox.points[1].x;
    float uly = obstacle->bbox.points[1].y;
    float urx = obstacle->bbox.points[2].x;
    float ury = obstacle->bbox.points[2].y;
    float lrx = obstacle->bbox.points[3].x;
    float lry = obstacle->bbox.points[3].y;

    for (size_t i = 0; i < x.size(); i++) {
      float d1 = norm(llx - x[i], lly - y[i]);
      float d2 = norm(ulx - x[i], uly - y[i]);
      float d3 = norm(urx - x[i], ury - y[i]);
      float d4 = norm(lrx - x[i], lry - y[i]);

      float closest = min({d1, d2, d3, d4});
      // only check for collision if one corner of bounding box is
      // within COLLISION_CHECK_THRESHOLD of waypoint
      if (closest <= COLLISION_CHECK_THRESHOLD) {
        float xp = static_cast<float>(x[i]);
        float yp = static_cast<float>(y[i]);
        float yawp = static_cast<float>(yaw[i]);
        pose.assign({xp, yp, yawp});
        car.setPose(pose);
        car_outline = car.getOutline();
        if (obstacle->isOverlap(car_outline)) {
          return true;
        }
      }
    }
  }

  // no collisions
  return false;
}

// calculate the sum of 1 / distance_to_obstacle
float FrenetPath::inverse_distance_to_obstacles(
    const vector<Obstacle *> obstacles) {
  float total_inverse_distance = 0.0;

  for (auto obstacle : obstacles) {
    float llx = obstacle->bbox.points[0].x;
    float lly = obstacle->bbox.points[0].y;
    float ulx = obstacle->bbox.points[1].x;
    float uly = obstacle->bbox.points[1].y;
    float urx = obstacle->bbox.points[2].x;
    float ury = obstacle->bbox.points[2].y;
    float lrx = obstacle->bbox.points[3].x;
    float lry = obstacle->bbox.points[3].y;

    for (size_t i = 0; i < x.size(); i++) {
      float d1 = norm(llx - x[i], lly - y[i]);
      float d2 = norm(ulx - x[i], uly - y[i]);
      float d3 = norm(urx - x[i], ury - y[i]);
      float d4 = norm(lrx - x[i], lry - y[i]);

      float closest = min({d1, d2, d3, d4});
      total_inverse_distance += static_cast<float>(1.0 / closest);
    }
  }
  return total_inverse_distance;
}
