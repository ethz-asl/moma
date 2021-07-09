void EndEffectorCost_intermediate_jacobian_sparsity(unsigned long const** row,
                                                    unsigned long const** col,
                                                    unsigned long* nnz) {
   static unsigned long const rows[55] = {0,1,2,3,4,5,6,7,8,9,9,9,9,9,9,9,9,10,10,10,10,10,10,10,10,11,11,11,11,11,11,12,12,12,12,12,12,12,12,13,13,13,13,13,13,13,13,14,14,14,14,14,14,14,14};
   static unsigned long const cols[55] = {11,12,13,14,15,16,17,18,19,1,3,4,5,6,7,8,9,2,3,4,5,6,7,8,9,4,5,6,7,8,9,3,4,5,6,7,8,9,10,3,4,5,6,7,8,9,10,3,4,5,6,7,8,9,10};
   *row = rows;
   *col = cols;
   *nnz = 55;
}
