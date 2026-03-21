#include "Obstacle.h"
#include "tool/recorder.h"

#include <QLine>

using namespace Eigen;
using namespace std;

Obstacle::Obstacle(Point_FP first_point, Point_FP second_point,
                   Point_FP third_point, Point_FP fourth_point) {
  bbox.points[0].x = first_point.x;
  bbox.points[0].y = first_point.y;
  bbox.points[1].x = second_point.x;
  bbox.points[1].y = second_point.y;
  bbox.points[2].x = third_point.x;
  bbox.points[2].y = third_point.y;
  bbox.points[3].x = fourth_point.x;
  bbox.points[3].y = fourth_point.y;
}

// Separating Axis Theorem Algorithmus
bool Obstacle::isOverlap(Rectangle &car_outline) {
  Rectangle obstacle = {{{bbox.points[0].x, bbox.points[0].y},
                         {bbox.points[1].x, bbox.points[1].y},
                         {bbox.points[2].x, bbox.points[2].y},
                         {bbox.points[3].x, bbox.points[3].y}}};

  Vector2D axes[4] = {
      perpendicular(subtract(car_outline.points[1], car_outline.points[0])),
      perpendicular(subtract(car_outline.points[2], car_outline.points[1])),
      perpendicular(subtract(obstacle.points[1], obstacle.points[0])),
      perpendicular(subtract(obstacle.points[2], obstacle.points[1]))};

  for (int i = 0; i < 4; i++) {
    if (isSeparated(car_outline, obstacle, axes[i])) {
      return false;
    }
  }
  return true;
}

bool Obstacle::isSeparated(Rectangle rect1, Rectangle rect2, Vector2D axis) {
  float min1, max1, min2, max2;

  min1 = max1 = dotProduct(rect1.points[0], axis);
  min2 = max2 = dotProduct(rect2.points[0], axis);

  for (int i = 1; i < 4; i++) {
    float projection = dotProduct(rect1.points[i], axis);
    if (projection < min1)
      min1 = projection;
    if (projection > max1)
      max1 = projection;

    projection = dotProduct(rect2.points[i], axis);
    if (projection < min2)
      min2 = projection;
    if (projection > max2)
      max2 = projection;
  }

  if (max1 < min2 || max2 < min1) {
    return true; // Separated
  }

  return false; // Not separated
}

Vector2D Obstacle::subtract(Vector2D a, Vector2D b) {
  Vector2D result;
  result.x = a.x - b.x;
  result.y = a.y - b.y;
  return result;
}

float Obstacle::dotProduct(Vector2D a, Vector2D b) {
  return static_cast<float>(a.x * b.x + a.y * b.y);
}

Vector2D Obstacle::perpendicular(Vector2D vector) {
  Vector2D result;
  result.x = -vector.y;
  result.y = vector.x;
  return result;
}
