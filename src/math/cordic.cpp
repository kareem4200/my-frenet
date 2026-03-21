//******************************************************************************
//   https://github.com/the0b/MSP430-CORDIC-sine-cosine
//   Description: This file contains a function to caluculate sine and cosine
//                using CORDIC rotation on a 16 bit MSP430. The function takes
//                any angle in degrees and outputs the answers in Q.15 fixed 
//                point format[(floating point) = (fixed point)/ 2^15]. 
//
//                In creating this file, I referenced the 2004 presentation of
//                fixed point two's complement CORDIC arithmetic presentation
//                by Titi Trandafir of Microtrend Systems which contained an 
//                assembly language program utilizing CORDIC. I also referenced
//                the CORDIC wikipedia page.
//
//   Theo Brower
//   Version    1.01
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
# include "cordic.h"
# include "math.h"
// Table of arctan's for use with CORDIC algorithm
// Store in decimal representation N = ((2^16)*angle_deg) / 180
int_2_13 atantable[ATAN_TAB_N]={  int_2_13(0.78539816),   //atan(2^0) =  0.78539816 rad
                                int_2_13(0.46364761),   //atan(2^-1) = 0.46364761 rad
                                int_2_13(0.24497866),   //atan(2^-2) = 0.24497866 rad
                                int_2_13(0.12435499),   //0.12435499 rad
                                int_2_13(0.06241881),   //0.06241881 rad
                                int_2_13(0.03123983),   //0.03123983 rad
                                int_2_13(0.01562373),   //0.01562373 rad
                                int_2_13(0.00781234),   //0.00781234 rad
                                int_2_13(0.00390623),   //0.00390623 rad
                                int_2_13(0.00195312),   //0.00195312 rad
                                int_2_13(0.00097656),   //0.00097656 rad
                                int_2_13(0.00048828),   //0.00048828 rad
                                int_2_13(0.00024414),   //0.00024414 rad
                                int_2_13(0.00012207)   //0.00012207 rad
};

int_3_12 normilize_yaw(int_3_12 theta)
{
  if(theta < int_3_12(-PI))
  {
    theta += ((-theta+int_3_12(PI))/int_3_12(2*PI))*int_3_12(2*PI);
  }
  if(theta > int_3_12(PI))
  {
    theta -= ((theta+int_3_12(PI))/int_3_12(2*PI))*int_3_12(2*PI);
  }
  return theta;
}

int_2_13 cordic_cos(int_3_12 theta)
{
  return cordic_cos_normalized(normilize_yaw(theta));
}

int_2_13 cordic_sin(int_3_12 theta)
{
  return cordic_sin_normalized(normilize_yaw(theta));
}

int_2_13 cordic_sin_normalized(int_2_13 theta){
  short sigma,quadAdj,i,shift;
  int_2_13 s, x1, x2, y;
  int_2_13 *atanptr = atantable;
  int_2_13 ans=0;
  char iterations=ITERATION;

  //Limit iterations to number of atan values in our table
  iterations = (iterations > ATAN_TAB_N) ? ATAN_TAB_N : iterations;

  //Shift angle to be in range -180 to 180
  while(theta < -PI) theta += 2.0*PI;
  while(theta > PI) theta -= 2.0*PI;
  
  //Shift angle to be in range -90 to 90
  if (theta < -PI_2){
    theta = theta + PI;
    quadAdj = -1;
  } else if (theta > PI_2){
    theta = theta - PI;
    quadAdj = -1;
  } else{
    quadAdj = 1;
  }

  //Shift angle to be in range -45 to 45
  if (theta < -PI_4){
    theta = theta + PI_2;
    shift = -1;
  } else if (theta > PI_4){
    theta = theta - PI_2;
    shift = 1;
  } else{
    shift = 0;
  }

  //Initial values
  x1 = magic_number;    //this will be the cosine result, 
                  //initially the magic number 0.60725293
  y = 0.0;          //y will contain the sine result
  s = 0.0;          //s will contain the final angle
  sigma = 1;      //direction from target angle
  
  for (i=0; i<iterations; i++){
    sigma = (theta - s) > 0 ? 1 : -1;
    if(sigma < 0){
      x2 = x1 + (y >> i);
      y = y - (x1 >> i);
      x1 = x2;
      s -= *atanptr++;
    } else{
      x2 = x1 - (y >> i);
      y = y + (x1 >> i);
      x1 = x2;
      s += *atanptr++;
    }
  }
  
  //Correct for possible overflow in cosine result
  if(x1 < 0) x1 = -x1;
  
  //Push final values to appropriate registers
  if(shift > 0){
    ans = x1;
  } else if (shift < 0){
    ans = -x1;
  } else {
    ans = y;
  }
  //Adjust for sign change if angle was in quadrant 3 or 4
  ans=quadAdj * ans;
  return ans;
}

int_2_13 cordic_cos_normalized(int_2_13 theta){
  short sigma,quadAdj,i,shift;
  int_2_13 s, x1, x2, y;
  int_2_13 *atanptr = atantable;
  int_2_13 ans=0;
  char iterations=ITERATION;
  
  //Limit iterations to number of atan values in our table
  iterations = (iterations > ATAN_TAB_N) ? ATAN_TAB_N : iterations;

  //Shift angle to be in range -180 to 180
  while(theta < -PI) theta += 2.0*PI;
  while(theta > PI) theta -= 2.0*PI;
  
  //Shift angle to be in range -90 to 90
  if (theta < -PI_2){
    theta = theta + PI;
    quadAdj = -1;
  } else if (theta > PI_2){
    theta = theta - PI;
    quadAdj = -1;
  } else{
    quadAdj = 1;
  }
  
  //Shift angle to be in range -45 to 45
  if (theta < -PI_4){
    theta = theta + PI_2;
    shift = -1;
  } else if (theta > PI_4){
    theta = theta - PI_2;
    shift = 1;
  } else{
    shift = 0;
  }
  
  //Initial values
  x1 = magic_number;    //this will be the cosine result, 
                  //initially the magic number 0.60725293
  y = 0;          //y will contain the sine result
  s = 0;          //s will contain the final angle
  sigma = 1;      //direction from target angle
  
  for (i=0; i<iterations; i++){
    sigma = (theta - s) > 0 ? 1 : -1;
    if(sigma < 0){
      x2 = x1 + (y >> i);
      y = y - (x1 >> i);
      x1 = x2;
      s -= *atanptr++;
    } else{
      x2 = x1 - (y >> i);
      y = y + (x1 >> i);
      x1 = x2;
      s += *atanptr++;
    }
  }
  
  //Correct for possible overflow in cosine result
  if(x1 < 0) x1 = -x1;
  
  //Push final values to appropriate registers
  if(shift > 0){
    ans = -y;
  } else if (shift < 0){
    ans = y;
  } else {
    ans = x1;
  }
  
  //Adjust for sign change if angle was in quadrant 3 or 4
  ans = quadAdj * ans;
  return ans;
}