//Implementation based on thomas algorithmn http://www.industrial-maths.com/ms6021_thomas.pdf
#include "TriDiagonalMatrixSolver.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif
#include <vector>

void solveTriDiagonalMatrix(const std::vector<float>& a, const std::vector<float>& b,
                                     const std::vector<float>& c, const std::vector<float>& d,
                                     std::vector<float>& result, short n) {
    std::vector<float> c_star(n, 0);
    std::vector<float> d_star(n, 0);

    // Forward sweep
    c_star[0] = c[0] / b[0];
    d_star[0] = d[0] / b[0];
    
    for (short i = 1; i < n; ++i) {
        float m = 1.0 / (b[i] - a[i] * c_star[i - 1]);
        c_star[i] = c[i] * m;
        d_star[i] = (d[i] - a[i] * d_star[i - 1]) * m;
        // #ifdef USE_RECORDER
        //     Recorder::getInstance()->saveData<float>("solveTriDiagonalMatrix::c_star", c_star[i]);
        //     Recorder::getInstance()->saveData<float>("solveTriDiagonalMatrix::d_star", d_star[i]);
        //     Recorder::getInstance()->saveData<float>("solveTriDiagonalMatrix::m", m);
        // #endif
    }

    // Backward sweep
    result.back() = d_star.back();
    for (short i = n - 1; i-- > 0; ) {
        result[i] = d_star[i] - c_star[i] * result[i + 1];
        // #ifdef USE_RECORDER
        //     Recorder::getInstance()->saveData<float>("solveTriDiagonalMatrix::result", result[i]);
        // #endif
    }
}
