//******************************************************************************
//
//   Description: Header file for CORDIC sine/cosine function, see .c file for
//                more detailed description.
//
//   T. Brower
//   Version    1.00
//   Feb 2011
//     IAR Embedded Workbench Kickstart (Version: 5.20.1)
//
//Copyright (c) 2011 Theo Brower
//
//Permission is hereby granted, free of charge, to any person obtaining a 
//copy of this software and associated documentation files (the "Software"), 
//to deal in the Software without restriction, including without limitation 
//the rights to use, copy, modify, merge, publish, distribute, sublicense, 
//and/or sell copies of the Software, and to permit persons to whom the 
//Software is furnished to do so, subject to the following conditions:
//The above copyright notice and this permission notice shall be included 
//in all copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS 
//OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
//THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
//FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
//IN THE SOFTWARE.
//******************************************************************************

#ifndef CORDIC_H
#define CORDIC_H

#include "cnl/all.h"
#include "tool/fp_datatype.h" 

#define ATAN_TAB_N 14
#define ITERATION 13
#define PI int_2_13(M_PI)
#define PI_2 int_2_13(M_PI_2)
#define PI_4 int_2_13(M_PI_4)
#define magic_number cnl::scaled_integer<int16_t, cnl::power<-13>>(0.60725293)

extern int_2_13 atantable[ATAN_TAB_N]; 

enum DataRange{
  under_range = 1,
  in_range = 2,
  abrove_range =3 
};

int_2_13  cordic_sin(int_3_12 theta);
int_2_13  cordic_cos(int_3_12  theta);
int_2_13  cordic_sin_normalized(int_2_13 theta);
int_2_13  cordic_cos_normalized(int_2_13 theta);
int_3_12  normilize_yaw(int_3_12 theta);


template<typename T>
int_2_13 cordic_atan(T y,  T x){
  int_2_13 z;
  T x1, x2, y_temp;
  int_2_13 *atanptr = atantable;
  char iterations=ITERATION;
  
  if (x == 0) {
    if (y > 0) return PI_2;
    if (y < 0) return -PI_2;
    return 0;
  }

  x1 = x;
  y_temp = y;
  
  //Initial angle
  z = 0;

  for (short i=0; i<iterations; i++){
    if(y_temp < 0){
      x2 = x1 - (y_temp >> i);
      y_temp = y_temp + (x1 >> i);
      x1 = x2;
      z -= *atanptr++;
    } else{
      x2 = x1 + (y_temp >> i);
      y_temp = y_temp - (x1 >> i);
      x1 = x2;
      z += *atanptr++;
    }
  }
  return z;
}

//https://de.mathworks.com/help/fixedpoint/ug/compute-square-root-using-cordic.html
template<typename T>
T cordic_sqrt(T input)
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

  int_2_13 x = number + int_2_13(1);
  int_2_13 y = number - int_2_13(1);

  // short k = 3;
  short n = 1;

  while(n <= ITERATION ){

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
    n++; 
  }
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

#endif /* CORDIC_H */