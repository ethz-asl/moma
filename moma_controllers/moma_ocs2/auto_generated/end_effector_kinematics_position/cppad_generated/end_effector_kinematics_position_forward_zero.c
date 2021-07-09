#include <math.h>
#include <stdio.h>

typedef struct Array {
    void* data;
    unsigned long size;
    int sparse;
    const unsigned long* idx;
    unsigned long nnz;
} Array;

struct LangCAtomicFun {
    void* libModel;
    int (*forward)(void* libModel,
                   int atomicIndex,
                   int q,
                   int p,
                   const Array tx[],
                   Array* ty);
    int (*reverse)(void* libModel,
                   int atomicIndex,
                   int p,
                   const Array tx[],
                   Array* px,
                   const Array py[]);
};

void end_effector_kinematics_position_forward_zero(double const *const * in,
                                                   double*const * out,
                                                   struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* y = out[0];

   // auxiliary variables
   double v[53];

   v[0] = cos(x[2]);
   v[1] = sin(x[2]);
   v[2] = 0 - v[1];
   v[3] = -2.76289999992544e-18 * v[0] + 7.34641020664359e-06 * v[2];
   v[4] = sin(x[3]);
   v[5] = 0 - v[4];
   v[6] = cos(x[3]);
   v[7] = v[5] + -2.02973967599405e-23 * v[6];
   v[8] = -4.9311988016995e-36 * v[5] + -0.999999999973015 * v[6];
   v[9] = v[0] * v[7] + v[2] * v[8];
   v[10] = v[6] + -2.02973967599405e-23 * v[4];
   v[11] = -4.9311988016995e-36 * v[6] + -0.999999999973015 * v[4];
   v[2] = v[0] * v[10] + v[2] * v[11];
   v[12] = -3.67320510363811e-06 * v[3] + -0.999999999993254 * v[9] + -1.11020078396468e-16 * v[2];
   v[13] = sin(x[4]);
   v[14] = 0 - v[13];
   v[15] = cos(x[4]);
   v[16] = -2.1343e-17 * v[14] + 0.999999999993254 * v[15];
   v[17] = -1.1102e-16 * v[14] + -3.67320510363811e-06 * v[15];
   v[14] = v[14] + 2.13425922006255e-17 * v[15];
   v[18] = v[3] * v[16] + v[9] * v[17] + v[2] * v[14];
   v[19] = -2.1343e-17 * v[15] + 0.999999999993254 * v[13];
   v[20] = -1.1102e-16 * v[15] + -3.67320510363811e-06 * v[13];
   v[15] = v[15] + 2.13425922006255e-17 * v[13];
   v[2] = v[3] * v[19] + v[9] * v[20] + v[2] * v[15];
   v[13] = -3.67320510363811e-06 * v[12] + 0.999999999993254 * v[18] + 2.91219999998035e-16 * v[2];
   v[21] = sin(x[5]);
   v[22] = 0 - v[21];
   v[23] = cos(x[5]);
   v[24] = -2.46519032881566e-32 * v[22] + -0.999999999993254 * v[23];
   v[25] = -2.9122e-16 * v[22] + -3.67320510363811e-06 * v[23];
   v[22] = v[22] + -1.06971079018608e-21 * v[23];
   v[26] = v[12] * v[24] + v[18] * v[25] + v[2] * v[22];
   v[27] = -2.46519032881566e-32 * v[23] + -0.999999999993254 * v[21];
   v[28] = -2.9122e-16 * v[23] + -3.67320510363811e-06 * v[21];
   v[23] = v[23] + -1.06971079018608e-21 * v[21];
   v[2] = v[12] * v[27] + v[18] * v[28] + v[2] * v[23];
   v[21] = -3.67320510363811e-06 * v[13] + -0.999999999993254 * v[26] + -1.66529754063102e-16 * v[2];
   v[29] = sin(x[6]);
   v[30] = 0 - v[29];
   v[31] = cos(x[6]);
   v[32] = 6.6954e-17 * v[30] + 0.999999999993254 * v[31];
   v[33] = -1.6653e-16 * v[30] + -3.67320510363811e-06 * v[31];
   v[30] = v[30] + -6.69546116983942e-17 * v[31];
   v[34] = v[13] * v[32] + v[26] * v[33] + v[2] * v[30];
   v[35] = 6.6954e-17 * v[31] + 0.999999999993254 * v[29];
   v[36] = -1.6653e-16 * v[31] + -3.67320510363811e-06 * v[29];
   v[31] = v[31] + -6.69546116983942e-17 * v[29];
   v[2] = v[13] * v[35] + v[26] * v[36] + v[2] * v[31];
   v[29] = -3.67320510363811e-06 * v[21] + 0.999999999993254 * v[34] + 6.37291844011089e-17 * v[2];
   v[37] = sin(x[7]);
   v[38] = 0 - v[37];
   v[39] = cos(x[7]);
   v[40] = -2.2204e-16 * v[38] + -0.999999999993254 * v[39];
   v[41] = -6.373e-17 * v[38] + -3.67320510363811e-06 * v[39];
   v[38] = v[38] + -2.22040234091863e-16 * v[39];
   v[42] = v[21] * v[40] + v[34] * v[41] + v[2] * v[38];
   v[43] = -2.2204e-16 * v[39] + -0.999999999993254 * v[37];
   v[44] = -6.373e-17 * v[39] + -3.67320510363811e-06 * v[37];
   v[39] = v[39] + -2.22040234091863e-16 * v[37];
   v[2] = v[21] * v[43] + v[34] * v[44] + v[2] * v[39];
   v[37] = -3.67320510363811e-06 * v[29] + -0.999999999993254 * v[42] + -8.21569999994458e-15 * v[2];
   v[45] = sin(x[8]);
   v[46] = 0 - v[45];
   v[47] = cos(x[8]);
   v[48] = -9.20600676392921e-28 * v[46] + 0.999999999993254 * v[47];
   v[49] = -8.2157e-15 * v[46] + -3.67320510363811e-06 * v[47];
   v[46] = v[46] + -3.01779502467892e-20 * v[47];
   v[50] = v[29] * v[48] + v[42] * v[49] + v[2] * v[46];
   v[51] = -9.20600676392921e-28 * v[47] + 0.999999999993254 * v[45];
   v[52] = -8.2157e-15 * v[47] + -3.67320510363811e-06 * v[45];
   v[47] = v[47] + -3.01779502467892e-20 * v[45];
   y[0] = -0.12838 * v[3] + 0.005375 * v[9] + x[0] + -0.006375 * v[12] + -0.21038 * v[18] + -0.21038 * v[13] + 0.006375 * v[26] + -0.006375 * v[21] + -0.20843 * v[34] + -0.10593 * v[29] + 0.00017505 * v[42] + -0.00017505 * v[37] + -0.10593 * v[50] + -0.0615250000000001 * (-3.67320510363811e-06 * v[37] + 0.999999999993254 * v[50] + -9.63957960960612e-17 * (v[29] * v[51] + v[42] * v[52] + v[2] * v[47]));
   v[50] = -2.76289999992544e-18 * v[1] + 7.34641020664359e-06 * v[0];
   v[8] = v[1] * v[7] + v[0] * v[8];
   v[11] = v[1] * v[10] + v[0] * v[11];
   v[10] = -3.67320510363811e-06 * v[50] + -0.999999999993254 * v[8] + -1.11020078396468e-16 * v[11];
   v[1] = v[50] * v[16] + v[8] * v[17] + v[11] * v[14];
   v[11] = v[50] * v[19] + v[8] * v[20] + v[11] * v[15];
   v[0] = -3.67320510363811e-06 * v[10] + 0.999999999993254 * v[1] + 2.91219999998035e-16 * v[11];
   v[7] = v[10] * v[24] + v[1] * v[25] + v[11] * v[22];
   v[11] = v[10] * v[27] + v[1] * v[28] + v[11] * v[23];
   v[37] = -3.67320510363811e-06 * v[0] + -0.999999999993254 * v[7] + -1.66529754063102e-16 * v[11];
   v[2] = v[0] * v[32] + v[7] * v[33] + v[11] * v[30];
   v[11] = v[0] * v[35] + v[7] * v[36] + v[11] * v[31];
   v[42] = -3.67320510363811e-06 * v[37] + 0.999999999993254 * v[2] + 6.37291844011089e-17 * v[11];
   v[29] = v[37] * v[40] + v[2] * v[41] + v[11] * v[38];
   v[11] = v[37] * v[43] + v[2] * v[44] + v[11] * v[39];
   v[34] = -3.67320510363811e-06 * v[42] + -0.999999999993254 * v[29] + -8.21569999994458e-15 * v[11];
   v[21] = v[42] * v[48] + v[29] * v[49] + v[11] * v[46];
   y[1] = -0.12838 * v[50] + 0.005375 * v[8] + x[1] + -0.006375 * v[10] + -0.21038 * v[1] + -0.21038 * v[0] + 0.006375 * v[7] + -0.006375 * v[37] + -0.20843 * v[2] + -0.10593 * v[42] + 0.00017505 * v[29] + -0.00017505 * v[34] + -0.10593 * v[21] + -0.0615250000000001 * (-3.67320510363811e-06 * v[34] + 0.999999999993254 * v[21] + -9.63957960960612e-17 * (v[42] * v[51] + v[29] * v[52] + v[11] * v[47]));
   v[5] = -2.7629e-18 * v[5] + -7.34641020664359e-06 * v[6];
   v[6] = -2.7629e-18 * v[6] + -7.34641020664359e-06 * v[4];
   v[4] = 3.67320510353899e-06 + -0.999999999993254 * v[5] + -1.11020078396468e-16 * v[6];
   v[14] = -0.999999999973015 * v[16] + v[5] * v[17] + v[6] * v[14];
   v[6] = -0.999999999973015 * v[19] + v[5] * v[20] + v[6] * v[15];
   v[15] = -3.67320510363811e-06 * v[4] + 0.999999999993254 * v[14] + 2.91219999998035e-16 * v[6];
   v[22] = v[4] * v[24] + v[14] * v[25] + v[6] * v[22];
   v[6] = v[4] * v[27] + v[14] * v[28] + v[6] * v[23];
   v[23] = -3.67320510363811e-06 * v[15] + -0.999999999993254 * v[22] + -1.66529754063102e-16 * v[6];
   v[30] = v[15] * v[32] + v[22] * v[33] + v[6] * v[30];
   v[6] = v[15] * v[35] + v[22] * v[36] + v[6] * v[31];
   v[31] = -3.67320510363811e-06 * v[23] + 0.999999999993254 * v[30] + 6.37291844011089e-17 * v[6];
   v[38] = v[23] * v[40] + v[30] * v[41] + v[6] * v[38];
   v[6] = v[23] * v[43] + v[30] * v[44] + v[6] * v[39];
   v[39] = -3.67320510363811e-06 * v[31] + -0.999999999993254 * v[38] + -8.21569999994458e-15 * v[6];
   v[46] = v[31] * v[48] + v[38] * v[49] + v[6] * v[46];
   y[2] = 0.284809999996536 + 0.005375 * v[5] + -0.006375 * v[4] + -0.21038 * v[14] + -0.21038 * v[15] + 0.006375 * v[22] + -0.006375 * v[23] + -0.20843 * v[30] + -0.10593 * v[31] + 0.00017505 * v[38] + -0.00017505 * v[39] + -0.10593 * v[46] + -0.0615250000000001 * (-3.67320510363811e-06 * v[39] + 0.999999999993254 * v[46] + -9.63957960960612e-17 * (v[31] * v[51] + v[38] * v[52] + v[6] * v[47]));
}

