#ifndef FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE1D_H
#define FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE1D_H

#include <vector>

// 1-dimensional cubic spline class.
// For technical details see: http://mathworld.wolfram.com/CubicSpline.html
class CubicSpline1D {
public:
    int nx;
    CubicSpline1D();
    CubicSpline1D(const std::vector<float>& v1, const std::vector<float>& v2);
    float calc_der0(float t);
    float calc_der1(float t);
    float calc_der2(float t);
private:
    std::vector<float> a, b, c, d, w, x, y;
    std::vector<float> tridionalmatrix_a, tridionalmatrix_b, tridionalmatrix_c, tridionalmatrix_d;
    int search_index(float t);
    void assignValue(std::vector<float> &TM_a, std::vector<float> &TM_b, 
            std::vector<float> &TM_c, std::vector<float> &TM_d, std::vector<float> &deltas);
};

#endif //FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE1D_H
