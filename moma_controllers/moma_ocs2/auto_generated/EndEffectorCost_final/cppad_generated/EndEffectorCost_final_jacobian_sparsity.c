void EndEffectorCost_final_jacobian_sparsity(unsigned long const** row,
                                             unsigned long const** col,
                                             unsigned long* nnz) {
   static unsigned long const rows[46] = {0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,2,2,2,2,2,2,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,5,5,5,5,5,5,5,5};
   static unsigned long const cols[46] = {1,3,4,5,6,7,8,9,2,3,4,5,6,7,8,9,4,5,6,7,8,9,3,4,5,6,7,8,9,10,3,4,5,6,7,8,9,10,3,4,5,6,7,8,9,10};
   *row = rows;
   *col = cols;
   *nnz = 46;
}
