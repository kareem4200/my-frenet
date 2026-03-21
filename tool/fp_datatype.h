#include "cnl/scaled_integer.h"

#define fixp_t uint_6_10
#define fixp_d int_3_12
#define fixp_d_d int_3_12
#define fixp_d_dd int_4_11
#define fixp_d_ddd int_7_8
#define fixp_s int_8_7
#define fixp_s_d int_5_10
#define fixp_s_dd int_5_10
#define fixp_s_ddd int_5_10
#define fixp_x int_8_7
// #define fixp_y int_3_8
#define fixp_y int_8_7
#define fixp_yaw int_3_12
#define fixp_ds int_3_12
#define fixp_c int_3_12
#define fixp_quarticpolynomial_A0 int_8_7
#define fixp_quarticpolynomial_A1_A4 int_4_11
#define fixp_quinticpolynomial_A0_A5 int_4_11
#define fixp_cost uint_8_8
#define fixp_lateral_jerk int_9_6
#define fixp_docProduct int_12_3
#define fixp_sum_cost uint_11_5
#define fixp_inverse_distanceToObstacles uint_6_10
#define fixp_mincost uint_11_5
#define fixp_quarticpolynomial_K1 int_6_9
#define fixp_quarticpolynomial_K2 int_2_13
#define fixp_quinticpolynomial_K int_5_10
#define fixp_quinticpolynomial_K0 int_5_10
#define fixp_quinticpolynomial_K1 int_2_13
#define fixp_quinticpolynomial_K2 int_2_13
#define fixp_quinticpolynomial_t2 int_6_9
#define fixp_quinticpolynomial_t3 int_8_7
#define fixp_quinticpolynomial_t4 int_11_4
#define fixp_quinticpolynomial_t5 int_13_2
#define fixp_cum_sum int_8_7

typedef cnl::scaled_integer<int16_t, cnl::power<-7>>  int_8_7;
typedef cnl::scaled_integer<int16_t, cnl::power<-8>>  int_7_8;
typedef cnl::scaled_integer<int16_t, cnl::power<-9>> int_6_9;
typedef cnl::scaled_integer<int16_t, cnl::power<-10>>  int_5_10;
typedef cnl::scaled_integer<int16_t, cnl::power<-13>> int_2_13;
typedef cnl::scaled_integer<int16_t, cnl::power<-11>> int_4_11;
typedef cnl::scaled_integer<int16_t, cnl::power<-12>> int_3_12;
typedef cnl::scaled_integer<int16_t, cnl::power<-14>> int_1_14;
typedef cnl::scaled_integer<int16_t, cnl::power<-6>> int_9_6;
typedef cnl::scaled_integer<int16_t, cnl::power<-4>> int_11_4;
typedef cnl::scaled_integer<int16_t, cnl::power<-3>> int_12_3;
typedef cnl::scaled_integer<int16_t, cnl::power<-2>> int_13_2;

typedef cnl::scaled_integer<uint16_t, cnl::power<-14>> uint_2_14;
typedef cnl::scaled_integer<uint16_t, cnl::power<-13>> uint_3_13;
typedef cnl::scaled_integer<uint16_t, cnl::power<-10>> uint_6_10;
typedef cnl::scaled_integer<uint16_t, cnl::power<-9>> uint_7_9;
typedef cnl::scaled_integer<uint16_t, cnl::power<-8>> uint_8_8;
typedef cnl::scaled_integer<uint16_t, cnl::power<-7>> uint_9_7;
typedef cnl::scaled_integer<uint16_t, cnl::power<-6>> uint_10_6;
typedef cnl::scaled_integer<uint16_t, cnl::power<-5>> uint_11_5;