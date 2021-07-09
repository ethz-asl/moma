void end_effector_kinematics_orientation_jacobian_sparsity(unsigned long const** row,
                                                           unsigned long const** col,
                                                           unsigned long* nnz) {
   static unsigned long const rows[24] = {0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2};
   static unsigned long const cols[24] = {2,3,4,5,6,7,8,9,2,3,4,5,6,7,8,9,2,3,4,5,6,7,8,9};
   *row = rows;
   *col = cols;
   *nnz = 24;
}
