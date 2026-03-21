#ifndef FRENET_OPTIMAL_TRAJECTORY_QUARTICPOLYNOMIAL_H
#define FRENET_OPTIMAL_TRAJECTORY_QUARTICPOLYNOMIAL_H

class QuarticPolynomial {
public:
    QuarticPolynomial() = default;
    QuarticPolynomial(float xs, float vxs, float axs, float vxe,
                      float axe, float t);
    float calc_point(float t);
    float calc_first_derivative(float t);
    float calc_second_derivative(float t);
    float calc_third_derivative(float t);
private:
    float a0; 
    float a1, a2, a3, a4;
};

#endif //FRENET_OPTIMAL_TRAJECTORY_QUARTICPOLYNOMIAL_H
