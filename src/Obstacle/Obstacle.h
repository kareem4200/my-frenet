#ifndef FRENETOPTIMALTRAJECTORY_OBSTACLE_H
#define FRENETOPTIMALTRAJECTORY_OBSTACLE_H

#include "utils.h"
#include <eigen3/Eigen/Dense>

using namespace Eigen;

class Obstacle {
public:
  Rectangle bbox;
  Obstacle(Point_FP first_point, Point_FP second_point, Point_FP third_point,
           Point_FP fourth_point);
  bool isOverlap(Rectangle &car_outline);
  float getArea();

private:
  Vector2D subtract(Vector2D a, Vector2D b);
  float dotProduct(Vector2D a, Vector2D b);
  Vector2D perpendicular(Vector2D vector);
  bool isSeparated(Rectangle rect1, Rectangle rect2, Vector2D axis);
};

#endif // FRENETOPTIMALTRAJECTORY_OBSTACLE_H
