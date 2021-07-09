void mobile_manipulator_dynamics_jump_map_jacobian_sparsity(unsigned long const** row,
                                                            unsigned long const** col,
                                                            unsigned long* nnz) {
   static unsigned long const rows[10] = {0,1,2,3,4,5,6,7,8,9};
   static unsigned long const cols[10] = {1,2,3,4,5,6,7,8,9,10};
   *row = rows;
   *col = cols;
   *nnz = 10;
}
