#include "QuarticPolynomial.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif

#include <cmath>

QuarticPolynomial::QuarticPolynomial(float xs, float vxs, float axs,
        float vxe, float axe, float t):
        a0(xs), a1(vxs) {
    a2 = axs / 2.0;
    float K1;
    float K2;
    K1=vxe-a1-2*a2*t;
    K2=axe-2*a2;
    a3 = K1/pow(t,2) - K2/(3*t);
    a4 = K2/(4*pow(t,2))-K1/(2*pow(t,3));
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<float>("QuarticPolynomial::QuarticPolynomial::xs", xs);
    //     Recorder::getInstance()->saveData<float>("QuarticPolynomial::QuarticPolynomial::vxs", vxs);
    //     Recorder::getInstance()->saveData<float>("QuarticPolynomial::QuarticPolynomial::axs", axs);
    //     Recorder::getInstance()->saveData<float>("QuarticPolynomial::QuarticPolynomial::vxe", vxe);
    //     Recorder::getInstance()->saveData<float>("QuarticPolynomial::QuarticPolynomial::axe", axe);
    //     Recorder::getInstance()->saveData<float>("QuarticPolynomial::QuarticPolynomial::t", t);
    //     Recorder::getInstance()->saveData<float>("QuarticPolynomial::QuarticPolynomial::K1", K1);
    //     Recorder::getInstance()->saveData<float>("QuarticPolynomial::QuarticPolynomial::K2", K2);
    //     Recorder::getInstance()->saveData<float>("QuarticPolynomial::QuarticPolynomial::a0", a0);
    //     Recorder::getInstance()->saveData<float>("QuarticPolynomial::QuarticPolynomial::a1", a1);
    //     Recorder::getInstance()->saveData<float>("QuarticPolynomial::QuarticPolynomial::a2", a2);
    //     Recorder::getInstance()->saveData<float>("QuarticPolynomial::QuarticPolynomial::a3", a3);
    //     Recorder::getInstance()->saveData<float>("QuarticPolynomial::QuarticPolynomial::a4", a4);
    // #endif
}

float QuarticPolynomial::calc_point(float t) {
    return a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3) + a4 * pow(t, 4);
}

float QuarticPolynomial::calc_first_derivative(float t) {
    return a1 + 2 * a2 * t + 3 * a3 * pow(t, 2) + 4 * a4 * pow(t, 3);
}

float QuarticPolynomial::calc_second_derivative(float t) {
    return 2 * a2 + 6 * a3 * t + 12 * a4 * pow(t, 2);
}

float QuarticPolynomial::calc_third_derivative(float t) {
    return 6 * a3 + 24 * a4 * t;
}