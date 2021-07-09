void mobile_manipulator_dynamics_flow_map_jacobian_sparsity(unsigned long const** row,
                                                            unsigned long const** col,
                                                            unsigned long* nnz) {
   static unsigned long const rows[12] = {0,0,1,1,2,3,4,5,6,7,8,9};
   static unsigned long const cols[12] = {3,11,3,11,12,13,14,15,16,17,18,19};
   *row = rows;
   *col = cols;
   *nnz = 12;
}
