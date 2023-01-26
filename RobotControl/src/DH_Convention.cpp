//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: DH_Convention.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-Oct-2015 15:23:40
//

// Include Files
//#include "rt_nonfinite.h"
#include "Drehmoment.h"
#include "DH_Convention.h"

// Function Definitions

//
// Arguments    : double q
//                double d
//                double a
//                double alpha
//                double A[16]
// Return Type  : void
//
void DH_Convention(double q, double d, double a, double alpha, double A[16])
{
  int i1;
  static const signed char iv0[4] = { 0, 0, 0, 1 };

  A[0] = cos(q);
  A[4] = -sin(q) * cos(alpha);
  A[8] = sin(q) * sin(alpha);
  A[12] = a * cos(q);
  A[1] = sin(q);
  A[5] = cos(q) * cos(alpha);
  A[9] = -cos(q) * sin(alpha);
  A[13] = a * sin(q);
  A[2] = 0.0;
  A[6] = sin(alpha);
  A[10] = cos(alpha);
  A[14] = d;
  for (i1 = 0; i1 < 4; i1++) {
    A[3 + (i1 << 2)] = iv0[i1];
  }
}

//
// File trailer for DH_Convention.cpp
//
// [EOF]
//
