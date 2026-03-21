#ifndef FRENET_OPTIMAL_TRAJECTORY_UTILS_H
#define FRENET_OPTIMAL_TRAJECTORY_UTILS_H

#include <cmath>
#include <tuple>
#include <vector>

using namespace std;

typedef struct { 
    float x,y;
} Point;

typedef struct { 
    float x;
    float y;
} Point_FP;

typedef struct {
    float x;
    float y;
} Vector2D;

typedef struct {
    Vector2D points[4];
} Rectangle;

typedef vector<float> Pose;

inline float norm(float x, float y) {
    return sqrt(pow(x, 2) + pow(y, 2));
}

// inline fixp_x norm_FP(fixp_x x, fixp_y y) {
//     return sqrt(static_cast<float>(x*x) + static_cast<float>(y*y));
// }

inline void as_unit_vector(tuple<float, float>& vec) {
    float magnitude = norm(get<0>(vec), get<1>(vec));
    if (magnitude > 0) {
        get<0>(vec) = get<0>(vec) / magnitude;
        get<1>(vec) = get<1>(vec) / magnitude;
    }
}

inline float dot(const tuple<float, float>& vec1,
                  const tuple<float, float>& vec2) {
    return get<0>(vec1) * get<0>(vec2) +
           get<1>(vec1) * get<1>(vec2);
}
#endif //FRENET_OPTIMAL_TRAJECTORY_UTILS_H
