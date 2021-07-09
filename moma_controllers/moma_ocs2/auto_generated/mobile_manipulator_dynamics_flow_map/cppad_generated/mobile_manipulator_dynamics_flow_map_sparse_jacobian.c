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

void mobile_manipulator_dynamics_flow_map_sparse_jacobian(double const *const * in,
                                                          double*const * out,
                                                          struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* jac = out[0];

   // auxiliary variables

   jac[0] = -1 * sin(x[3]) * x[11];
   jac[1] = cos(x[3]);
   jac[2] = cos(x[3]) * x[11];
   jac[3] = sin(x[3]);
   // dependent variables without operations
   jac[4] = 1;
   jac[5] = 1;
   jac[6] = 1;
   jac[7] = 1;
   jac[8] = 1;
   jac[9] = 1;
   jac[10] = 1;
   jac[11] = 1;
}

