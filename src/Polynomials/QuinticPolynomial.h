#ifndef FRENET_OPTIMAL_TRAJECTORY_QUINTICPOLYNOMIAL_H
#define FRENET_OPTIMAL_TRAJECTORY_QUINTICPOLYNOMIAL_H

class QuinticPolynomial {
public:
    QuinticPolynomial() = default;
    QuinticPolynomial(float xs, float vxs, float axs, float xe,
                      float vxe, float axe, float t);
    float calc_point(float t);
    float calc_first_derivative(float t);
    float calc_second_derivative(float t);
    float calc_third_derivative(float t);
private:
    float a0, a1, a2, a3, a4, a5;
};

#endif //FRENET_OPTIMAL_TRAJECTORY_QUINTICPOLYNOMIAL_H
