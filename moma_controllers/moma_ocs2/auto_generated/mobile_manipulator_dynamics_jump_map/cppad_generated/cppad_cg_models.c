void cppad_cg_models(char const *const** names,
                     int* count) {
   static const char* const models[] = {
      "mobile_manipulator_dynamics_jump_map"};
   *names = models;
   *count = 1;
}
