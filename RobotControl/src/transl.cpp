//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: transl.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-Oct-2015 15:23:40
//

// Include Files
//#include "../jaco_force/rt_nonfinite.h"
#include "Drehmoment.h"
#include "transl.h"

// Function Definitions

//
// Arguments    : const double pos[3]
//                double T[16]
// Return Type  : void
//
void transl(const double pos[3], double T[16])
{
  int i3;
  static const signed char iv3[4] = { 0, 0, 0, 1 };

  T[0] = 1.0;
  T[4] = 0.0;
  T[8] = 0.0;
  T[12] = pos[0];
  T[1] = 0.0;
  T[5] = 1.0;
  T[9] = 0.0;
  T[13] = pos[1];
  T[2] = 0.0;
  T[6] = 0.0;
  T[10] = 1.0;
  T[14] = pos[2];
  for (i3 = 0; i3 < 4; i3++) {
    T[3 + (i3 << 2)] = iv3[i3];
  }
}

//
// File trailer for transl.cpp
//
// [EOF]
//
