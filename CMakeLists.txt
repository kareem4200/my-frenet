cmake_minimum_required(VERSION 3.10)
project(FrenetOptimalTrajectory)


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "-march=native -O3 -pthread -Wall -DQT_NO_VERSION_TAGGING")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

# if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)" )
#   set(CNL_DIR "${CNL_DIR} /home/wenguang/installed/cnl/include")
# elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "(amd64)|(AMD64)|(aarch64)")
#   set(CNL_DIR "${CNL_DIR} /space/xuw/Installed/cnl/include")
# endif()


find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Eigen3)

option(USE_RECORDER "Use recorder" OFF)
option(ANALYZE_VECTORIZATION "Detect if vectorization is successful" OFF)
option(SAMPLING_PATH_ANALYSIS "Return sampling paths for analysis" OFF)

set(recorder_file ${recorder_file})
if (USE_RECORDER)
  MESSAGE(STATUS "USE_RECORDER=ON")
  add_definitions(-DUSE_RECORDER)
  set(recorder_file ${recorder_file} tool/recorder.h tool/recorder.cpp tool/fp_datatype.h)
endif()

if (ANALYZE_VECTORIZATION)
  MESSAGE(STATUS "ANALYZE_VECTORIZATION=ON")
  add_definitions(-DANALYZE_VECTORIZATION)
  set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-Rpass-analysis=loop-vectorize")
endif()

if (SAMPLING_PATH_ANALYSIS)
  MESSAGE(STATUS "SAMPLING_PATH_ANALYSIS=ON")
  add_definitions(-DSAMPLING_PATH_ANALYSIS)
endif()

include_directories(.)
include_directories(
        ${CUSTOM_EIGEN}
)
# include_directories(${CNL_DIR})

include_directories("${CMAKE_SOURCE_DIR}/src"
                    "${CMAKE_SOURCE_DIR}/src/CubicSpline"
                    "${CMAKE_SOURCE_DIR}/src/Polynomials"
                    "${CMAKE_SOURCE_DIR}/src/FrenetOptimalTrajectory"
                    "${CMAKE_SOURCE_DIR}/src/Obstacle"
                    "${CMAKE_SOURCE_DIR}/src/Car"
                    "${CMAKE_SOURCE_DIR}/src/math")

add_library(FrenetOptimalTrajectory SHARED
            src/utils.h
            src/Polynomials/QuarticPolynomial.cpp
            src/Polynomials/QuarticPolynomial.h
            src/Polynomials/QuinticPolynomial.cpp
            src/Polynomials/QuinticPolynomial.h
            src/CubicSpline/CubicSpline1D.cpp
            src/CubicSpline/CubicSpline1D.h
            src/CubicSpline/CubicSpline2D.cpp
            src/CubicSpline/CubicSpline2D.h
            src/CubicSpline/TriDiagonalMatrixSolver.h
            src/CubicSpline/TriDiagonalMatrixSolver.cpp
            src/FrenetOptimalTrajectory/FrenetOptimalTrajectory.cpp
            src/FrenetOptimalTrajectory/FrenetOptimalTrajectory.h
            src/FrenetOptimalTrajectory/FrenetPath.cpp
            src/FrenetOptimalTrajectory/FrenetPath.h
            src/FrenetOptimalTrajectory/fot_wrapper.cpp
            src/FrenetOptimalTrajectory/py_cpp_struct.h
            src/Obstacle/Obstacle.cpp
            src/Obstacle/Obstacle.h
            src/Car/Car.cpp
            src/Car/Car.h
            ${recorder_file}
            # src/math/cordic.cpp
            # src/math/cordic.h
            )

add_executable(FrenetOptimalTrajectoryTest
               src/main.cpp
               src/utils.h
               src/Polynomials/QuarticPolynomial.cpp
               src/Polynomials/QuarticPolynomial.h
               src/Polynomials/QuinticPolynomial.cpp
               src/Polynomials/QuinticPolynomial.h
               src/CubicSpline/CubicSpline1D.cpp
               src/CubicSpline/CubicSpline1D.h
               src/CubicSpline/CubicSpline2D.cpp
               src/CubicSpline/CubicSpline2D.h
               src/CubicSpline/TriDiagonalMatrixSolver.h
               src/CubicSpline/TriDiagonalMatrixSolver.cpp
               src/FrenetOptimalTrajectory/FrenetOptimalTrajectory.cpp
               src/FrenetOptimalTrajectory/FrenetOptimalTrajectory.h
               src/FrenetOptimalTrajectory/FrenetPath.cpp
               src/FrenetOptimalTrajectory/FrenetPath.h
               src/FrenetOptimalTrajectory/fot_wrapper.cpp
               src/FrenetOptimalTrajectory/py_cpp_struct.h
               src/Obstacle/Obstacle.cpp
               src/Obstacle/Obstacle.h
               src/Car/Car.cpp
               src/Car/Car.h
               ${recorder_file}
              #  src/math/cordic.cpp
              #  src/math/cordic.h
               )
target_link_libraries(FrenetOptimalTrajectory Eigen3::Eigen Qt5::Widgets Qt5::Gui)
target_link_libraries(FrenetOptimalTrajectoryTest Eigen3::Eigen Qt5::Widgets Qt5::Gui)
