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

void mobile_manipulator_dynamics_jump_map_forward_zero(double const *const * in,
                                                       double*const * out,
                                                       struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* y = out[0];

   // auxiliary variables

   // dependent variables without operations
   y[0] = x[1];
   y[1] = x[2];
   y[2] = x[3];
   y[3] = x[4];
   y[4] = x[5];
   y[5] = x[6];
   y[6] = x[7];
   y[7] = x[8];
   y[8] = x[9];
   y[9] = x[10];
}

