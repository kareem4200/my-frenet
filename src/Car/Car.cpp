#include "Car.h"

// Set the pose of the car
void Car::setPose(Pose p) {
    pose = p;
}

// Compute the outline of the car given its current pose
Rectangle Car::getOutline() {
    float x, y, yaw;
    float tail_x, tail_y, head_x, head_y;
    Vector2D tail_l, tail_r;
    Vector2D head_l, head_r;

    x = pose[0];
    y = pose[1];
    yaw = pose[2];

    tail_x = x - cos(yaw) * length * 0.5;
    tail_y = y - sin(yaw) * length * 0.5;
    tail_l.x = tail_x + cos(yaw + M_PI_2) * width / 2.0;
    tail_l.y = tail_y + sin(yaw + M_PI_2) * width / 2.0;
    tail_r.x = tail_x + cos(yaw - M_PI_2) * width / 2.0;
    tail_r.y = tail_y + sin(yaw - M_PI_2) * width / 2.0;

    head_x = x + cos(yaw) * length * 0.5;
    head_y = y + sin(yaw) * length * 0.5;
    head_l.x = head_x + cos(yaw + M_PI_2) * width / 2.0;
    head_l.y = head_y + sin(yaw + M_PI_2) * width / 2.0;
    head_r.x = head_x + cos(yaw - M_PI_2) * width / 2.0;
    head_r.y = head_y + sin(yaw - M_PI_2) * width / 2.0;

    Rectangle outline ={tail_l, tail_r, head_r, head_l};
    return outline;
}