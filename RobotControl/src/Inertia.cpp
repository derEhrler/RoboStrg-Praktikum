//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Inertia.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-Oct-2015 15:23:40
//

// Include Files
//#include "../jaco_ik/rt_nonfinite.h"
#include "Drehmoment.h"
#include "Inertia.h"
#include "transl.h"
#include "trotz.h"
#include "DH_Convention.h"
#include "cross.h"

// Function Definitions

//
// Arguments    : const double q[6]
//                double B[36]
// Return Type  : void
//
void Inertia(const double q[6], double B[36])
{
  double d[6];
  double a[6];
  double alpha[6];
  double offset[6];
  double I_Link[54];
  int i4;
  int j;
  static const double dv5[9] = { 324893.60122828, -4.14134763, 5.21439353,
    -4.14134763, 353694.43212892, 21186.54126343, 5.21439353, 21186.54126343,
    102070.25511638 };

  static const double dv6[9] = { 69776.66770227, 59.03745106, 0.0, 59.03745106,
    3.7251264465396E+6, 0.0, 0.0, 0.0, 3.78623046078145E+6 };

  static const double dv7[9] = { 48101.3603617, -1.96023437, -1.92269188,
    -1.96023437, 510995.83838128, -1586.22384686, -1.92269188, -1586.22384686,
    480446.67577576 };

  static const double dv8[9] = { 36318.26, 0.34, 0.35, 0.34, 34174.42, -8328.79,
    0.35, -8328.79, 24561.75 };

  static const double dv9[9] = { 6318.26, 0.34, -0.35, 0.34, 34174.42, 8328.79,
    -0.35, 8328.79, 24561.75 };

  static const double dv10[9] = { 52479.87, -105.73, -32.44, -105.73, 46596.24,
    -4130.32, -32.44, -4130.32, 26473.06 };

  double I_Motor[54];
  static const double dv11[9] = { 243715.84, 43.29, -75.61, 43.29, 244186.05,
    -251.41, -75.61, -251.41, 276812.76 };

  static const double dv12[9] = { 238984.81, 172.35, 245.07, 172.35, 238840.01,
    104.4, 245.07, 104.4, 273005.59 };

  static const double dv13[9] = { 244186.05, 43.44, 251.41, 43.29, 243715.84,
    75.61, 251.41, 75.61, 276812.76 };

  static const double dv14[9] = { 119312.98, -232.8, 23.94, -232.8, 119711.58,
    -4.92, 23.94, -4.92, 119866.5 };

  double T[96];
  double A[96];
  double T_Link[96];
  double T_Motor[96];
  int i;
  double dv15[16];
  double dv16[16];
  static const double P_Link[18] = { -0.00023712254999999998,
    -0.010291609969999999, 0.06917216347, -2.629337E-5, -0.20523664355,
    -0.02230776575, -0.06437250505, 7.073408000000001E-5, -0.029368740859999996,
    0.0, 0.00861, -0.24484, 0.0, -0.00881, -0.080321654946951274,
    2.9999999999999997E-5, -0.00684, -0.12500165494695126 };

  double b_T[16];
  int i5;
  static const signed char b_a[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 1 };

  static const double P_Motor[18] = { -2.0E-5, 1.0E-5, -0.00856, -2.0E-5, 2.0E-5,
    0.0080600000000000012, -2.0E-5, 1.0E-5, 0.00856, -2.9999999999999997E-5,
    -2.0E-5, -0.20164, -2.9999999999999997E-5, -2.0E-5, -0.037121654946951264,
    -2.9999999999999997E-5, -2.0E-5, -0.037121654946951264 };

  double J_Pos_Link[108];
  double J_Ori_Link[108];
  double J_Pos_Motor[108];
  double J_Ori_Motor[108];
  double b_T_Motor[3];
  static const double dv17[3] = { 0.0, 0.0, 1.0 };

  double b_J_Ori_Link[18];
  double c_J_Ori_Link[18];
  double d_J_Ori_Link[18];
  double b_B[36];
  double e_J_Ori_Link[36];
  double d2;
  static const double dv18[6] = { 0.18198400092, 0.42399300239000004,
    0.21100391393999998, 0.06896, 0.06896, 0.727 };

  double b_J_Ori_Motor[18];
  double c_B[36];
  static const double dv19[6] = { 0.57368, 0.55972, 0.57368, 0.34141000000000005,
    0.34141000000000005, 0.34141000000000005 };

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
  //  Massentraegheismoment in Bezug auf Link-Koordinatensystem
  memset(&I_Link[0], 0, 54U * sizeof(double));
  for (i4 = 0; i4 < 3; i4++) {
    for (j = 0; j < 3; j++) {
      I_Link[j + 3 * i4] = dv5[j + 3 * i4];
      I_Link[9 + (j + 3 * i4)] = dv6[j + 3 * i4];
      I_Link[18 + (j + 3 * i4)] = dv7[j + 3 * i4];
      I_Link[27 + (j + 3 * i4)] = dv8[j + 3 * i4];
      I_Link[36 + (j + 3 * i4)] = dv9[j + 3 * i4];
      I_Link[45 + (j + 3 * i4)] = dv10[j + 3 * i4];
    }
  }

  for (i4 = 0; i4 < 54; i4++) {
    // Massentragheistmoment von Motor
    I_Motor[i4] = 0.0;
    I_Link[i4] /= 1.0E+9;
  }

  for (i4 = 0; i4 < 3; i4++) {
    for (j = 0; j < 3; j++) {
      I_Motor[j + 3 * i4] = dv11[j + 3 * i4];
      I_Motor[9 + (j + 3 * i4)] = dv12[j + 3 * i4];
      I_Motor[18 + (j + 3 * i4)] = dv13[j + 3 * i4];
      I_Motor[27 + (j + 3 * i4)] = dv14[j + 3 * i4];
      I_Motor[36 + (j + 3 * i4)] = dv14[j + 3 * i4];
      I_Motor[45 + (j + 3 * i4)] = dv14[j + 3 * i4];
    }
  }

  for (i4 = 0; i4 < 54; i4++) {
    I_Motor[i4] /= 1.0E+9;
  }

  // % Transformationsmatrix
  memset(&T[0], 0, 96U * sizeof(double));
  for (i = 0; i < 6; i++) {
    if (1 + i == 1) {
      DH_Convention(q[i] + offset[i], d[i], a[i], alpha[i], *(double (*)[16])&
                    A[i << 4]);
      trotz(q[i], dv15);
      transl(*(double (*)[3])&P_Link[3 * i], dv16);
      for (i4 = 0; i4 < 4; i4++) {
        for (j = 0; j < 4; j++) {
          T[(i4 + (j << 2)) + (i << 4)] = 0.0;
          for (i5 = 0; i5 < 4; i5++) {
            T[(i4 + (j << 2)) + (i << 4)] += (double)b_a[i4 + (i5 << 2)] * A[(i5
              + (j << 2)) + (i << 4)];
          }

          b_T[i4 + (j << 2)] = 0.0;
          for (i5 = 0; i5 < 4; i5++) {
            b_T[i4 + (j << 2)] += (double)b_a[i4 + (i5 << 2)] * dv15[i5 + (j <<
              2)];
          }
        }

        for (j = 0; j < 4; j++) {
          T_Link[(i4 + (j << 2)) + (i << 4)] = 0.0;
          for (i5 = 0; i5 < 4; i5++) {
            T_Link[(i4 + (j << 2)) + (i << 4)] += b_T[i4 + (i5 << 2)] * dv16[i5
              + (j << 2)];
          }
        }
      }

      trotz(q[i], dv15);
      transl(*(double (*)[3])&P_Motor[3 * i], dv16);
      for (i4 = 0; i4 < 4; i4++) {
        for (j = 0; j < 4; j++) {
          b_T[i4 + (j << 2)] = 0.0;
          for (i5 = 0; i5 < 4; i5++) {
            b_T[i4 + (j << 2)] += (double)b_a[i4 + (i5 << 2)] * dv15[i5 + (j <<
              2)];
          }
        }

        for (j = 0; j < 4; j++) {
          T_Motor[(i4 + (j << 2)) + (i << 4)] = 0.0;
          for (i5 = 0; i5 < 4; i5++) {
            T_Motor[(i4 + (j << 2)) + (i << 4)] += b_T[i4 + (i5 << 2)] * dv16[i5
              + (j << 2)];
          }
        }
      }
    } else {
      DH_Convention(q[i] + offset[i], d[i], a[i], alpha[i], *(double (*)[16])&
                    A[i << 4]);
      for (i4 = 0; i4 < 4; i4++) {
        for (j = 0; j < 4; j++) {
          b_T[i4 + (j << 2)] = 0.0;
          for (i5 = 0; i5 < 4; i5++) {
            b_T[i4 + (j << 2)] += T[(i4 + (i5 << 2)) + ((i - 1) << 4)] * A[(i5 +
              (j << 2)) + (i << 4)];
          }
        }
      }

      for (i4 = 0; i4 < 4; i4++) {
        for (j = 0; j < 4; j++) {
          T[(j + (i4 << 2)) + (i << 4)] = b_T[j + (i4 << 2)];
        }
      }

      trotz(q[i], dv15);
      transl(*(double (*)[3])&P_Link[3 * i], dv16);
      for (i4 = 0; i4 < 4; i4++) {
        for (j = 0; j < 4; j++) {
          b_T[i4 + (j << 2)] = 0.0;
          for (i5 = 0; i5 < 4; i5++) {
            b_T[i4 + (j << 2)] += T[(i4 + (i5 << 2)) + ((i - 1) << 4)] * dv15[i5
              + (j << 2)];
          }
        }

        for (j = 0; j < 4; j++) {
          T_Link[(i4 + (j << 2)) + (i << 4)] = 0.0;
          for (i5 = 0; i5 < 4; i5++) {
            T_Link[(i4 + (j << 2)) + (i << 4)] += b_T[i4 + (i5 << 2)] * dv16[i5
              + (j << 2)];
          }
        }
      }

      trotz(q[i], dv15);
      transl(*(double (*)[3])&P_Motor[3 * i], dv16);
      for (i4 = 0; i4 < 4; i4++) {
        for (j = 0; j < 4; j++) {
          b_T[i4 + (j << 2)] = 0.0;
          for (i5 = 0; i5 < 4; i5++) {
            b_T[i4 + (j << 2)] += T[(i4 + (i5 << 2)) + ((i - 1) << 4)] * dv15[i5
              + (j << 2)];
          }
        }

        for (j = 0; j < 4; j++) {
          T_Motor[(i4 + (j << 2)) + (i << 4)] = 0.0;
          for (i5 = 0; i5 < 4; i5++) {
            T_Motor[(i4 + (j << 2)) + (i << 4)] += b_T[i4 + (i5 << 2)] * dv16[i5
              + (j << 2)];
          }
        }
      }
    }
  }

  // % Jacobi_Matrix
  for (i4 = 0; i4 < 108; i4++) {
    J_Pos_Link[i4] = 0.0;

    //  Position       Jacobimatrix von Link
    J_Ori_Link[i4] = 0.0;

    //  Orientierung   Jacobimatrix von Link
    J_Pos_Motor[i4] = 0.0;

    //  Positionsm
    J_Ori_Motor[i4] = 0.0;
  }

  for (i = 0; i < 6; i++) {
    for (j = 0; j <= i; j++) {
      if (1 + j == 1) {
        for (i4 = 0; i4 < 3; i4++) {
          b_T_Motor[i4] = T_Link[12 + (i4 + (i << 4))];
          J_Ori_Link[(i4 + 3 * j) + 18 * i] = dv17[i4];
        }

        cross(dv17, b_T_Motor, *(double (*)[3])&J_Pos_Link[3 * j + 18 * i]);
      } else {
        for (i4 = 0; i4 < 3; i4++) {
          b_T_Motor[i4] = T_Link[12 + (i4 + (i << 4))] - T[12 + (i4 + ((j - 1) <<
            4))];
        }

        cross(*(double (*)[3])&T[8 + ((j - 1) << 4)], b_T_Motor, *(double (*)[3])
              &J_Pos_Link[3 * j + 18 * i]);
        for (i4 = 0; i4 < 3; i4++) {
          J_Ori_Link[(i4 + 3 * j) + 18 * i] = T[8 + (i4 + ((j - 1) << 4))];
        }
      }
    }
  }

  for (i = 0; i < 6; i++) {
    for (j = 0; j <= i; j++) {
      if ((j + 1 < 1 + i) && (j + 1 == 1)) {
        for (i4 = 0; i4 < 3; i4++) {
          b_T_Motor[i4] = T_Motor[12 + (i4 + (i << 4))];
        }

        cross(dv17, b_T_Motor, *(double (*)[3])&J_Pos_Motor[3 * j + 18 * i]);
      } else {
        if (j + 1 < 1 + i) {
          for (i4 = 0; i4 < 3; i4++) {
            b_T_Motor[i4] = T_Motor[12 + (i4 + (i << 4))] - T[12 + (i4 + ((j - 1)
              << 4))];
          }

          cross(*(double (*)[3])&T[8 + ((j - 1) << 4)], b_T_Motor, *(double (*)
                 [3])&J_Pos_Motor[3 * j + 18 * i]);
        }
      }
    }
  }

  for (i = 0; i < 6; i++) {
    for (j = 0; j <= i; j++) {
      if (1 + j < 1 + i) {
        for (i4 = 0; i4 < 3; i4++) {
          J_Ori_Motor[(i4 + 3 * j) + 18 * i] = J_Ori_Link[(i4 + 3 * j) + 18 * i];
        }
      } else {
        for (i4 = 0; i4 < 3; i4++) {
          J_Ori_Motor[(i4 + 3 * j) + 18 * i] = T_Motor[8 + (i4 + (i << 4))];
        }
      }
    }
  }

  memset(&B[0], 0, 36U * sizeof(double));
  for (i = 0; i < 6; i++) {
    for (i4 = 0; i4 < 6; i4++) {
      for (j = 0; j < 3; j++) {
        b_J_Ori_Link[i4 + 6 * j] = 0.0;
        for (i5 = 0; i5 < 3; i5++) {
          b_J_Ori_Link[i4 + 6 * j] += J_Ori_Link[(i5 + 3 * i4) + 18 * i] *
            T_Link[(i5 + (j << 2)) + (i << 4)];
        }
      }

      for (j = 0; j < 3; j++) {
        c_J_Ori_Link[i4 + 6 * j] = 0.0;
        for (i5 = 0; i5 < 3; i5++) {
          c_J_Ori_Link[i4 + 6 * j] += b_J_Ori_Link[i4 + 6 * i5] * I_Link[(i5 + 3
            * j) + 9 * i];
        }
      }

      for (j = 0; j < 3; j++) {
        d_J_Ori_Link[i4 + 6 * j] = 0.0;
        for (i5 = 0; i5 < 3; i5++) {
          d_J_Ori_Link[i4 + 6 * j] += c_J_Ori_Link[i4 + 6 * i5] * T_Link[(i5 +
            (j << 2)) + (i << 4)];
        }
      }
    }

    for (i4 = 0; i4 < 6; i4++) {
      for (j = 0; j < 6; j++) {
        d2 = 0.0;
        for (i5 = 0; i5 < 3; i5++) {
          d2 += dv18[i] * J_Pos_Link[(i5 + 3 * i4) + 18 * i] * J_Pos_Link[(i5 +
            3 * j) + 18 * i];
        }

        b_B[i4 + 6 * j] = B[i4 + 6 * j] + d2;
      }
    }

    for (i4 = 0; i4 < 6; i4++) {
      for (j = 0; j < 6; j++) {
        e_J_Ori_Link[i4 + 6 * j] = 0.0;
        for (i5 = 0; i5 < 3; i5++) {
          e_J_Ori_Link[i4 + 6 * j] += d_J_Ori_Link[i4 + 6 * i5] * J_Ori_Link[(i5
            + 3 * j) + 18 * i];
        }
      }

      for (j = 0; j < 3; j++) {
        b_J_Ori_Link[i4 + 6 * j] = 0.0;
        for (i5 = 0; i5 < 3; i5++) {
          b_J_Ori_Link[i4 + 6 * j] += J_Ori_Motor[(i5 + 3 * i4) + 18 * i] *
            T_Motor[(i5 + (j << 2)) + (i << 4)];
        }
      }

      for (j = 0; j < 3; j++) {
        c_J_Ori_Link[i4 + 6 * j] = 0.0;
        for (i5 = 0; i5 < 3; i5++) {
          c_J_Ori_Link[i4 + 6 * j] += b_J_Ori_Link[i4 + 6 * i5] * I_Motor[(i5 +
            3 * j) + 9 * i];
        }
      }

      for (j = 0; j < 3; j++) {
        b_J_Ori_Motor[i4 + 6 * j] = 0.0;
        for (i5 = 0; i5 < 3; i5++) {
          b_J_Ori_Motor[i4 + 6 * j] += c_J_Ori_Link[i4 + 6 * i5] * T_Motor[(i5 +
            (j << 2)) + (i << 4)];
        }
      }
    }

    for (i4 = 0; i4 < 6; i4++) {
      for (j = 0; j < 6; j++) {
        d2 = 0.0;
        for (i5 = 0; i5 < 3; i5++) {
          d2 += dv19[i] * J_Pos_Motor[(i5 + 3 * i4) + 18 * i] * J_Pos_Motor[(i5
            + 3 * j) + 18 * i];
        }

        c_B[i4 + 6 * j] = (b_B[i4 + 6 * j] + e_J_Ori_Link[i4 + 6 * j]) + d2;
      }
    }

    for (i4 = 0; i4 < 6; i4++) {
      for (j = 0; j < 6; j++) {
        b_B[i4 + 6 * j] = 0.0;
        for (i5 = 0; i5 < 3; i5++) {
          b_B[i4 + 6 * j] += b_J_Ori_Motor[i4 + 6 * i5] * J_Ori_Motor[(i5 + 3 *
            j) + 18 * i];
        }
      }
    }

    for (i4 = 0; i4 < 6; i4++) {
      for (j = 0; j < 6; j++) {
        B[j + 6 * i4] = c_B[j + 6 * i4] + b_B[j + 6 * i4];
      }
    }
  }
}

//
// File trailer for Inertia.cpp
//
// [EOF]
//
