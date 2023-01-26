//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Drehmoment.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-Oct-2015 15:23:40
//

// Include Files
//#include "rt_nonfinite.h"
#include "Drehmoment.h"
#include "transl.h"
#include "trotz.h"
#include "DH_Convention.h"
#include "cross.h"
#include "Inertia.h"

// Function Definitions

//
// Arguments    : const double q[6]
//                const double qd[6]
//                const double qdd[6]
//                double Torque[6]
// Return Type  : void
//
void Drehmoment(double q[6], double qd[6], double qdd[6],
                double Torque[6])
{
  double d[6];
  double a[6];
  double alpha[6];
  double offset[6];
  double T[96];
  double A[96];
  double T_Link[96];
  double T_Motor[96];
  int i;
  double dv0[16];
  double dv1[16];
  static const double P_Link[18] = { -0.00023712254999999998,
    -0.010291609969999999, 0.06917216347, -2.629337E-5, -0.20523664355,
    -0.02230776575, -0.06437250505, 7.073408000000001E-5, -0.029368740859999996,
    0.0, 0.00861, -0.24484, 0.0, -0.00881, -0.080321654946951274,
    2.9999999999999997E-5, -0.00684, -0.12500165494695126 };

  double b_T[16];
  int i0;
  int k;
  int j;
  static const signed char b_a[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 1 };

  static const double P_Motor[18] = { -2.0E-5, 1.0E-5, -0.00856, -2.0E-5, 2.0E-5,
    0.0080600000000000012, -2.0E-5, 1.0E-5, 0.00856, -2.9999999999999997E-5,
    -2.0E-5, -0.20164, -2.9999999999999997E-5, -2.0E-5, -0.037121654946951264,
    -2.9999999999999997E-5, -2.0E-5, -0.037121654946951264 };

  double J_Pos_Link[108];
  double J_Pos_Motor[108];
  double b_T_Motor[3];
  static const double dv2[3] = { 0.0, 0.0, 1.0 };

  double d0;
  double d1;
  static const double b[3] = { 0.0, 0.0, -9.81 };

  static const double dv3[6] = { 0.18198400092, 0.42399300239000004,
    0.21100391393999998, 0.06896, 0.06896, 0.727 };

  static const double dv4[6] = { 0.57368, 0.55972, 0.57368, 0.34141000000000005,
    0.34141000000000005, 0.34141000000000005 };

  double B[36];
  double CM[216];
  double dBdq[216];
  double b_B[6];
  static const signed char temp[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  double C[36];
  double b_C[6];
  static const double M_offset[6] = { -0.9, 0.4, -0.45, 0.15, -1.5, -0.25 };

  //  Umgebung
  //  Roboter Parameter
  //  DH-Parameter von Roboter
  d[0] = 0.2755;
  a[0] = 0.0;
  alpha[0] = 1.5707963267948966;
  offset[0] = 0.0;
  d[1] = 0.0;
  a[1] = 0.41;
  alpha[1] = 3.1415926535897931;
  offset[1] = -1.5707963267948966;
  d[2] = -0.0098;
  a[2] = 0.0;
  alpha[2] = 1.5707963267948966;
  offset[2] = 1.5707963267948966;
  d[3] = -0.25008165494695128;
  a[3] = 0.0;
  alpha[3] = 1.0471975511965976;
  offset[3] = 0.0;
  d[4] = -0.085563309893902526;
  a[4] = 0.0;
  alpha[4] = 1.0471975511965976;
  offset[4] = -3.1415926535897931;
  d[5] = -0.20278165494695127;
  a[5] = 0.0;
  alpha[5] = 3.1415926535897931;
  offset[5] = 1.5707963267948966;

  //  Die Position von Schwerpunkt
  //  Masse von Link
  // 150
  //  Masse von Motor
  // % Transformationsmatrix
  memset(&T[0], 0, 96U * sizeof(double));
  for (i = 0; i < 6; i++) {
    if (1 + i == 1) {
      DH_Convention(q[i] + offset[i], d[i], a[i], alpha[i], *(double (*)[16])&
                    A[i << 4]);
      trotz(q[i], dv0);
      transl(*(double (*)[3])&P_Link[3 * i], dv1);
      for (i0 = 0; i0 < 4; i0++) {
        for (k = 0; k < 4; k++) {
          T[(i0 + (k << 2)) + (i << 4)] = 0.0;
          for (j = 0; j < 4; j++) {
            T[(i0 + (k << 2)) + (i << 4)] += (double)b_a[i0 + (j << 2)] * A[(j +
              (k << 2)) + (i << 4)];
          }

          b_T[i0 + (k << 2)] = 0.0;
          for (j = 0; j < 4; j++) {
            b_T[i0 + (k << 2)] += (double)b_a[i0 + (j << 2)] * dv0[j + (k << 2)];
          }
        }

        for (k = 0; k < 4; k++) {
          T_Link[(i0 + (k << 2)) + (i << 4)] = 0.0;
          for (j = 0; j < 4; j++) {
            T_Link[(i0 + (k << 2)) + (i << 4)] += b_T[i0 + (j << 2)] * dv1[j +
              (k << 2)];
          }
        }
      }

      trotz(q[i], dv0);
      transl(*(double (*)[3])&P_Motor[3 * i], dv1);
      for (i0 = 0; i0 < 4; i0++) {
        for (k = 0; k < 4; k++) {
          b_T[i0 + (k << 2)] = 0.0;
          for (j = 0; j < 4; j++) {
            b_T[i0 + (k << 2)] += (double)b_a[i0 + (j << 2)] * dv0[j + (k << 2)];
          }
        }

        for (k = 0; k < 4; k++) {
          T_Motor[(i0 + (k << 2)) + (i << 4)] = 0.0;
          for (j = 0; j < 4; j++) {
            T_Motor[(i0 + (k << 2)) + (i << 4)] += b_T[i0 + (j << 2)] * dv1[j +
              (k << 2)];
          }
        }
      }
    } else {
      DH_Convention(q[i] + offset[i], d[i], a[i], alpha[i], *(double (*)[16])&
                    A[i << 4]);
      for (i0 = 0; i0 < 4; i0++) {
        for (k = 0; k < 4; k++) {
          b_T[i0 + (k << 2)] = 0.0;
          for (j = 0; j < 4; j++) {
            b_T[i0 + (k << 2)] += T[(i0 + (j << 2)) + ((i - 1) << 4)] * A[(j +
              (k << 2)) + (i << 4)];
          }
        }
      }

      for (i0 = 0; i0 < 4; i0++) {
        for (k = 0; k < 4; k++) {
          T[(k + (i0 << 2)) + (i << 4)] = b_T[k + (i0 << 2)];
        }
      }

      trotz(q[i], dv0);
      transl(*(double (*)[3])&P_Link[3 * i], dv1);
      for (i0 = 0; i0 < 4; i0++) {
        for (k = 0; k < 4; k++) {
          b_T[i0 + (k << 2)] = 0.0;
          for (j = 0; j < 4; j++) {
            b_T[i0 + (k << 2)] += T[(i0 + (j << 2)) + ((i - 1) << 4)] * dv0[j +
              (k << 2)];
          }
        }

        for (k = 0; k < 4; k++) {
          T_Link[(i0 + (k << 2)) + (i << 4)] = 0.0;
          for (j = 0; j < 4; j++) {
            T_Link[(i0 + (k << 2)) + (i << 4)] += b_T[i0 + (j << 2)] * dv1[j +
              (k << 2)];
          }
        }
      }

      trotz(q[i], dv0);
      transl(*(double (*)[3])&P_Motor[3 * i], dv1);
      for (i0 = 0; i0 < 4; i0++) {
        for (k = 0; k < 4; k++) {
          b_T[i0 + (k << 2)] = 0.0;
          for (j = 0; j < 4; j++) {
            b_T[i0 + (k << 2)] += T[(i0 + (j << 2)) + ((i - 1) << 4)] * dv0[j +
              (k << 2)];
          }
        }

        for (k = 0; k < 4; k++) {
          T_Motor[(i0 + (k << 2)) + (i << 4)] = 0.0;
          for (j = 0; j < 4; j++) {
            T_Motor[(i0 + (k << 2)) + (i << 4)] += b_T[i0 + (j << 2)] * dv1[j +
              (k << 2)];
          }
        }
      }
    }
  }

  // % Jacobi_Matrix
  for (i0 = 0; i0 < 108; i0++) {
    J_Pos_Link[i0] = 0.0;

    //  Position       Jacobimatrix von Link
    //  Orientierung   Jacobimatrix von Link
    J_Pos_Motor[i0] = 0.0;
  }

  //  Positionsm
  for (i = 0; i < 6; i++) {
    for (j = 0; j <= i; j++) {
      if (1 + j == 1) {
        for (i0 = 0; i0 < 3; i0++) {
          b_T_Motor[i0] = T_Link[12 + (i0 + (i << 4))];
        }

        cross(dv2, b_T_Motor, *(double (*)[3])&J_Pos_Link[3 * j + 18 * i]);
      } else {
        for (i0 = 0; i0 < 3; i0++) {
          b_T_Motor[i0] = T_Link[12 + (i0 + (i << 4))] - T[12 + (i0 + ((j - 1) <<
            4))];
        }

        cross(*(double (*)[3])&T[8 + ((j - 1) << 4)], b_T_Motor, *(double (*)[3])
              &J_Pos_Link[3 * j + 18 * i]);
      }
    }
  }

  for (i = 0; i < 6; i++) {
    for (j = 0; j <= i; j++) {
      if ((j + 1 < 1 + i) && (j + 1 == 1)) {
        for (i0 = 0; i0 < 3; i0++) {
          b_T_Motor[i0] = T_Motor[12 + (i0 + (i << 4))];
        }

        cross(dv2, b_T_Motor, *(double (*)[3])&J_Pos_Motor[3 * j + 18 * i]);
      } else {
        if (j + 1 < 1 + i) {
          for (i0 = 0; i0 < 3; i0++) {
            b_T_Motor[i0] = T_Motor[12 + (i0 + (i << 4))] - T[12 + (i0 + ((j - 1)
              << 4))];
          }

          cross(*(double (*)[3])&T[8 + ((j - 1) << 4)], b_T_Motor, *(double (*)
                 [3])&J_Pos_Motor[3 * j + 18 * i]);
        }
      }
    }
  }

  // % Matrix
  for (i = 0; i < 6; i++) {
    d[i] = 0.0;
  }

  for (i = 0; i < 6; i++) {
    for (j = 0; j < 6; j++) {
      d0 = 0.0;
      d1 = 0.0;
      for (i0 = 0; i0 < 3; i0++) {
        d0 += dv3[j] * b[i0] * J_Pos_Link[(i0 + 3 * i) + 18 * j];
        d1 += dv4[j] * b[i0] * J_Pos_Motor[(i0 + 3 * i) + 18 * j];
      }

      d[i] -= d0 + d1;
    }
  }

  Inertia(q, B);
  memset(&CM[0], 0, 216U * sizeof(double));
  for (i = 0; i < 6; i++) {
    for (i0 = 0; i0 < 6; i0++) {
      b_B[i0] = q[i0] + (double)temp[i + 6 * i0] * 0.0001;
    }

    Inertia(b_B, *(double (*)[36])&dBdq[36 * i]);
  }

  for (i = 0; i < 6; i++) {
    for (j = 0; j < 6; j++) {
      for (k = 0; k < 6; k++) {
        if ((1 + i == 1 + j) && (1 + j == 1 + k)) {
          CM[(i + 6 * j) + 36 * k] = 0.0;
        } else if (1 + j > 1 + k) {
          CM[(i + 6 * j) + 36 * k] = CM[(i + 6 * k) + 36 * j];
        } else {
          CM[(i + 6 * j) + 36 * k] = 0.5 * (((dBdq[(i + 6 * j) + 36 * k] - B[i +
            6 * j]) / 0.0001 + (dBdq[(i + 6 * k) + 36 * j] - B[i + 6 * k]) /
            0.0001) - (dBdq[(j + 6 * k) + 36 * i] - B[j + 6 * k]) / 0.0001);
        }
      }
    }
  }

  memset(&C[0], 0, 36U * sizeof(double));

  // % Drehmoment
  for (i = 0; i < 6; i++) {
    for (j = 0; j < 6; j++) {
      for (k = 0; k < 6; k++) {
        C[i + 6 * j] += CM[(i + 6 * j) + 36 * k] * qd[k];
      }
    }

    b_B[i] = 0.0;
    for (i0 = 0; i0 < 6; i0++) {
      b_B[i] += B[i + 6 * i0] * qdd[i0];
    }

    b_C[i] = 0.0;
    for (i0 = 0; i0 < 6; i0++) {
      b_C[i] += C[i + 6 * i0] * qd[i0];
    }

    Torque[i] = ((b_B[i] + b_C[i]) + d[i]) + M_offset[i];
  }
}

//
// File trailer for Drehmoment.cpp
//
// [EOF]
//
