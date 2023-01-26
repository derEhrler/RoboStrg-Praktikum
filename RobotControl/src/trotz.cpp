//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: trotz.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-Oct-2015 15:23:40
//

// Include Files
//#include "../jaco_force/rt_nonfinite.h"
#include "Drehmoment.h"
#include "trotz.h"

// Function Definitions

//
// Arguments    : double alpha
//                double A[16]
// Return Type  : void
//
void trotz(double alpha, double A[16])
{
  int i2;
  static const signed char iv1[4] = { 0, 0, 1, 0 };

  static const signed char iv2[4] = { 0, 0, 0, 1 };

  A[0] = cos(alpha);
  A[4] = -sin(alpha);
  A[8] = 0.0;
  A[12] = 0.0;
  A[1] = sin(alpha);
  A[5] = cos(alpha);
  A[9] = 0.0;
  A[13] = 0.0;
  for (i2 = 0; i2 < 4; i2++) {
    A[2 + (i2 << 2)] = iv1[i2];
    A[3 + (i2 << 2)] = iv2[i2];
  }
}

//
// File trailer for trotz.cpp
//
// [EOF]
//
