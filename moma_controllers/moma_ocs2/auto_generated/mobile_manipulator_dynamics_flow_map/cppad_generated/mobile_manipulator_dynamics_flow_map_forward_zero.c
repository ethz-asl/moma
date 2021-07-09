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

void mobile_manipulator_dynamics_flow_map_forward_zero(double const *const * in,
                                                       double*const * out,
                                                       struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* y = out[0];

   // auxiliary variables

   y[0] = cos(x[3]) * x[11];
   y[1] = sin(x[3]) * x[11];
   // dependent variables without operations
   y[2] = x[12];
   y[3] = x[13];
   y[4] = x[14];
   y[5] = x[15];
   y[6] = x[16];
   y[7] = x[17];
   y[8] = x[18];
   y[9] = x[19];
}

