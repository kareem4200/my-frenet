/*
How to compile this file:
clang++ -isystem /home/wenguang/installed/cnl/include -std=c++20 CNL_Test.cpp -o CNL_Test
clang++ -isystem /home/wenguang/installed/cnl_Debug/include -std=c++20 CNL_Test.cpp ../src/math/cordic.cpp -o CNL_Test
*/
#include "cnl/all.h"
// #include "/home/wenguang/workplace/frenet_optimal_trajectory_planner/src/math/cordic.h"
#include <cmath>
#include <iostream>
#include <limits>
using cnl::power;
using cnl::scaled_integer;
typedef cnl::scaled_integer<uint16_t, cnl::power<-14>> uint_2_14;
typedef cnl::scaled_integer<int16_t, cnl::power<-13>> int_2_13;
typedef cnl::scaled_integer<uint16_t, cnl::power<-7>> uint_9_7;

enum DataRange{
  under_range = 1,
  in_range = 2,
  abrove_range =3 
};

// float test_sqrt(float number) { //https://de.mathworks.com/help/fixedpoint/ug/compute-square-root-using-cordic.html
//   short f=0;
//   if(number<0.5)
//   {
//     short factor=1;
//     float u = number*pow(2,2*factor);;
//     while (u<0.5)
//     {
//       factor++;
//       float u = number*pow(2,2*factor);
//       if(u>=0.5)
//       {
//         f=factor;
//         break;
//       } 
//     }
//   }
//   else if(number>=2)
//   {
//     short factor=-1;
//     float u = number*pow(2,2*factor);
//     while (u>=2)
//     {
//       factor--;
//       float u = number*pow(2,2*factor);
//       if(u<2)
//       {
//         f=factor;
//         break;
//       }
//     }
//   }

//   number = number*pow(2,2*f);

//   float x = number+1;
//   float y = number-1;
//   float z = 0; 

//   float k = 3;
//   float n = 1;

//   while(n <= 20 ){

//     float xn = pow(2.0,-1.0*n) * y;
//     float yn = pow(2.0,-1.0*n) * x;

//     if(y < 0){ 
//         x = x + xn;
//         y = y + yn; 
//     }
//     else
//     {
//         x = x - xn;
//         y = y - yn;
//     }

//     if(n !=4 && n!=13){
//         //k = k-1;
//     }
//     else{
//       xn = pow(2.0,-1.0*n) * y;   // recalculate!
//       yn = pow(2.0,-1.0*n) * x;
//         //k = 3;
//         if(y < 0){ 
//             x = x + xn;
//             y = y + yn;
//         }
//         else
//         {
//             x = x - xn;
//             y = y - yn;
//         }
//     }
//     n++; 
//   }
//   return x/2*1.207497*pow(2,-f);;
// }

//https://de.mathworks.com/help/fixedpoint/ug/compute-square-root-using-cordic.html
template<typename T>
T test_sqrt_fxp(T input)
{
  T ans;
  DataRange range = DataRange::in_range;
  short factor=0;
  uint_2_14 number=input; 
  if(input<0.5)
  {
    range = DataRange::under_range;
    factor=1;
    T u = input<<(2*factor);;
    while (u<0.5)
    {
      factor++;
      u = input<<(2*factor);
      if(u>=0.5)
      {
        break;
      } 
    }
    number = input<<(2*factor);
  }
  else if(input>=2.0)
  {
    range = DataRange::abrove_range;
    factor=1;
    T u = input>>(2*factor);
    while (u>=2)
    {
      factor++;
      u = input>>(2*factor);
      if(u<2)
      {
        break;
      }
    }
    number = input>>(2*factor);
  }

  std::cout << "number= " << number << std::endl;
  int_2_13 x = number + int_2_13(1);
  int_2_13 y = number - int_2_13(1);

  short k = 3;
  short n = 1;

  while(n <= 20 ){

    int_2_13 xn = y>>n;
    int_2_13 yn = x>>n;

    if(y < 0){ 
        x = x + xn;
        y = y + yn; 
    }
    else
    {
        x = x - xn;
        y = y - yn;
    }

    if(n !=4 && n!=13){
        //k = k-1;
    }
    else{
      xn = y>>n;   // recalculate!
      yn = x>>n;
        //k = 3;
        if(y < 0){ 
            x = x + xn;
            y = y + yn;
        }
        else
        {
            x = x - xn;
            y = y - yn;
        }
    }
    std::cout <<"x_fix = "<<x<< std::endl;
    std::cout <<"y_fix = "<<y<< std::endl;
    n++; 
  }
  std::cout << "factor = " << factor << std::endl;
  if(range == DataRange::abrove_range)
  {
    ans = x<<(factor-1);
    ans *= 1.207497;
  }
  else 
  {
    ans = x>>(factor+1);
    ans *= 1.207497;
  }
  return ans;
}

float test_sqrt(float number) { 
  short f=0;
  if(number<0.5)
  {
    short factor=1;
    float u = number*pow(2,2*factor);;
    while (u<0.5)
    {
      factor++;
      float u = number*pow(2,2*factor);
      if(u>=0.5)
      {
        f=factor;
        break;
      } 
    }
  }
  else if(number>=2)
  {
    short factor=-1;
    float u = number*pow(2,2*factor);
    while (u>=2)
    {
      factor--;
      float u = number*pow(2,2*factor);
      if(u<2)
      {
        f=factor;
        break;
      }
    }
  }

  number = number*pow(2,2*f);
  std::cout << "number= " << number << std::endl;
  float x = number+1;
  float y = number-1;
  float z = 0; 

  float k = 3;
  float n = 1;

  while(n <= 20 ){

    float xn = pow(2.0,-1.0*n) * y;
    float yn = pow(2.0,-1.0*n) * x;

    if(y < 0){ 
        x = x + xn;
        y = y + yn; 
    }
    else
    {
        x = x - xn;
        y = y - yn;
    }

    if(n !=4 && n!=13){
        //k = k-1;
    }
    else{
      xn = pow(2.0,-1.0*n) * y;   // recalculate!
      yn = pow(2.0,-1.0*n) * x;
        //k = 3;
        if(y < 0){ 
            x = x + xn;
            y = y + yn;
        }
        else
        {
            x = x - xn;
            y = y - yn;
        }
    }
    std::cout <<"x_float = "<<x<< std::endl;
    std::cout <<"y_float = "<<y<< std::endl;
    n++; 
  }
  std::cout <<"f = "<<f<< std::endl;
  return x/2*1.207497*pow(2,-f);;
}

int main() {
    // cnl::scaled_integer<int16_t, cnl::power<-13>> a;
    // cnl::scaled_integer<int16_t, cnl::power<-13>> b;
    // cnl::scaled_integer<int16_t, cnl::power<-13>> ans;
    // std::cout << std::numeric_limits<
    //                  cnl::scaled_integer<int16_t, cnl::power<-12>>>::max()
    //           << std::endl;
    // a = 2.512;
    // cnl::scaled_integer<int16_t, cnl::power<-13>> a=cnl::scaled_integer<int16_t, cnl::power<-13>>(0.5);
    float a=0.005;
    int_2_13 b=a;
    std::cout << test_sqrt_fxp<int_2_13>(b) <<std::endl;
    std::cout << "-------------------------------" << std::endl;
    std::cout << test_sqrt(a) << std::endl;
    // a=30.0/180.0*M_PI;
    // ans=cordic_sin(a);
    // std::cout << "sin(" <<  a<< ") = " << ans << std::endl;
    // ans=cordic_cos(a);
    // std::cout << "cos(" <<  a<< ") = " << ans << std::endl;
    // a=45.0/180.0*M_PI;
    // ans=cordic_sin(a);
    // std::cout << "sin(" <<  a<< ") = " << ans << std::endl;
    // ans=cordic_cos(a);
    // std::cout << "cos(" <<  a<< ") = " << ans << std::endl;
    // a=90.0/180.0*M_PI;
    // ans=cordic_sin(a);
    // std::cout << "sin(" <<  a<< ") = " << ans << std::endl;
    // ans=cordic_cos(a);
    // std::cout << "cos(" <<  a<< ") = " << ans << std::endl;
}