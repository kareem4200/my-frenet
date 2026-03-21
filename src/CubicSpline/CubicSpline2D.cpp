#include "CubicSpline2D.h"
#include "utils.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif

#include <algorithm>
#include <numeric>
#include <cmath>

using namespace std;

// Default constructor
CubicSpline2D::CubicSpline2D() = default;

// Construct the 2-dimensional cubic spline
CubicSpline2D::CubicSpline2D(const vector<float> &x,
                             const vector<float> &y) {
    vector<vector<float>> filtered_points = remove_collinear_points(x, y);
    calc_s(filtered_points[0], filtered_points[1]);
    sx = CubicSpline1D(s, filtered_points[0]);
    sy = CubicSpline1D(s, filtered_points[1]);
}

// Calculate the s values for interpolation given x, y
void CubicSpline2D::calc_s(const vector<float>& x,
                           const vector<float>& y) {
    int nx = x.size();
    vector<float> dx (nx);//delta_x
    vector<float> dy (nx);
    adjacent_difference(x.begin(), x.end(), dx.begin());
    adjacent_difference(y.begin(), y.end(), dy.begin());
    dx.erase(dx.begin());
    dy.erase(dy.begin());

    float cum_sum = 0.0;
    s.push_back(cum_sum);
    for (int i = 0; i < nx - 1; i++) {
        cum_sum += norm(dx[i], dy[i]);
        // #ifdef USE_RECORDER
        //     Recorder::getInstance()->saveData<float>("CubicSpline2D::calc_s::cum_sum", cum_sum);
        // #endif
        s.push_back(cum_sum);
    }
    s.erase(unique(s.begin(), s.end()), s.end());
}

// Calculate the x position along the spline at given t
float CubicSpline2D::calc_x(float t) {
    return sx.calc_der0(t);
}

// Calculate the y position along the spline at given t
float CubicSpline2D::calc_y(float t) {
    return sy.calc_der0(t);
}

// Calculate the curvature along the spline at given t
float CubicSpline2D::calc_curvature(float t){
    float dx = sx.calc_der1(t);
    float ddx = sx.calc_der2(t);
    float dy = sy.calc_der1(t);
    float ddy = sy.calc_der2(t);
    float k = (ddy * dx - ddx * dy) /
            pow(pow(dx, 2) + pow(dy, 2), 1.5);
    // #ifdef USE_RECORDER
    //         Recorder::getInstance()->saveData<float>("CubicSpline2D::calc_curvature::dx", dx);
    //         Recorder::getInstance()->saveData<float>("CubicSpline2D::calc_curvature::ddx", ddx);
    //         Recorder::getInstance()->saveData<float>("CubicSpline2D::calc_curvature::dy", dy);
    //         Recorder::getInstance()->saveData<float>("CubicSpline2D::calc_curvature::ddy", ddy);
    //         Recorder::getInstance()->saveData<float>("CubicSpline2D::calc_curvature::k", k);
    // #endif
    return k;
}

// Calculate the yaw along the spline at given t
float CubicSpline2D::calc_yaw(float t) {
    float dx = sx.calc_der1(t);
    float dy = sy.calc_der1(t);
    float yaw = atan2(dy, dx);
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<float>("CubicSpline2D::calc_yaw::dx", dx);
    //     Recorder::getInstance()->saveData<float>("CubicSpline2D::calc_yaw::dy", dy);
    //     Recorder::getInstance()->saveData<float>("CubicSpline2D::calc_yaw::yaw", yaw);
    // #endif
    return yaw;
}

// Given x, y positions and an initial guess s0, find the closest s value
float CubicSpline2D::find_s(float x, float y, float s0) {
    float s_closest = s0;
    float closest = INFINITY;
    float si = s.front();

    do {
        float px = calc_x(si);
        float py = calc_y(si);
        float dist = norm(x - px, y - py);
        if (dist < closest) {
            closest = dist;
            s_closest = si;
        }
        si += 0.1;
        // #ifdef USE_RECORDER
        //     Recorder::getInstance()->saveData<float>("CubicSpline2D::find_s::px", px);
        //     Recorder::getInstance()->saveData<float>("CubicSpline2D::find_s::py", py);
        //     Recorder::getInstance()->saveData<float>("CubicSpline2D::find_s::s_closest", s_closest);
        //     Recorder::getInstance()->saveData<float>("CubicSpline2D::find_s::closest", closest);
        //     Recorder::getInstance()->saveData<float>("CubicSpline2D::find_s::dist", dist);
        // #endif
    } while (si < s.back());
    return s_closest;
}

// Remove any collinear points from given list of points by the triangle rule
vector<vector<float>>
CubicSpline2D::remove_collinear_points(vector<float> x, vector<float> y) {
    vector<vector<float>> filtered_points;
    vector<float> x_, y_;
    x_.push_back(static_cast<float>(x[0]));
    y_.push_back(static_cast<float>(y[0]));
    int delete_point_num = 0;
    for (size_t i = 2; i < x.size()-1; i++) {
        bool collinear = are_collinear(
            x[i - 2-delete_point_num], y[i - 2-delete_point_num],
            x[i - 1], y[i - 1],
            x[i], y[i]
            );
        if (collinear) {
            delete_point_num++;
            continue;
        }
        if(delete_point_num!=0)
        {
            x_.push_back(x[i-1]);
            y_.push_back(y[i-1]);
        }
        x_.push_back(static_cast<float>(x[i]));
        y_.push_back(static_cast<float>(y[i]));
        delete_point_num=0;
    }
    // make sure to add the last point in case all points are collinear
    x_.push_back(x.back());
    y_.push_back(y.back());
    filtered_points.push_back(x_);
    filtered_points.push_back(y_);
    return filtered_points;
}

// Determine if 3 points are collinear using the triangle area rule
bool CubicSpline2D::are_collinear(float x1, float y1, float x2, float y2,
    float x3, float y3) {
    float a =  abs(x1 * (y2 - y3) +
               x2 * (y3 - y1) +
               x3 * (y1 - y2));
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<float>("CubicSpline2D::are_collinear::a", a);
    // #endif
    return a <= 0.01;
}
