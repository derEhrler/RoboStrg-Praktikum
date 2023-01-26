//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Drehmoment.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-Oct-2015 15:23:40
//
#ifndef __DREHMOMENT_H__
#define __DREHMOMENT_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "Drehmoment_types.h"

// Function Declarations
extern "C"{
extern void Drehmoment(const double q[6], const double qd[6], const double qdd[6],
  double Torque[6]);
}

#endif

//
// File trailer for Drehmoment.h
//
// [EOF]
//
