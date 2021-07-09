void end_effector_kinematics_velocity_info(const char** baseName,
                                           unsigned long* m,
                                           unsigned long* n,
                                           unsigned int* indCount,
                                           unsigned int* depCount) {
   *baseName = "double  d";
   *m = 3;
   *n = 19;
   *depCount = 1; // number of dependent array variables
   *indCount = 1; // number of independent array variables
}

