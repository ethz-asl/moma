# piloting_demo

### Simulation

In order to run the simulation the SMB stack needs to be cloned. This is reflected in the `moma_piloting.repos` which should be merged using vcstool.

1. Merge the package dependencies with vcstool from the `src` directory of your catking workspace
    ```
    vcs import --input moma/moma_piloting.repos
    ```

2. Update dependencies of the smb stack through rosdep: 

    ```
    rosdep install --from-paths src --ignore-src --skip-keys="pinocchio" -r -y
    ```
    We skip `pinocchio` as we rely on a custom installation which enables the collision detection. Refer to [the installation script](../../install_dependencies.sh) for further information on the installation procedure.

3. At the moment, rosdep does not find some of the packages required in the smb stack. Then it is necessary to iterate the build process and install all missing packages with `sudo apt-get install ros-noetic-<pkgname>` until all packages were resolved. This is an annoying procedure, that we could improve changing the smb stack at a later stage.

4. Build the `piloting_demo` package (see above point if something goes wrong)
launch simulation (arm + base + localization + planner + base control):     
    ```
    roslaunch piloting_demo superpanda_sim.launch
    ```
    The last step should spawn something like this in rviz (note the additional navigation pluing):

    ![alt text](../docs/navigation_demo.png)

    Make sure the correct odom frame is set in the navigation panel. After that, one can click on Edit to move the 2d interactive marker to set a new base pose, and after that Start Planning to generate a global path which is then followed by the base (tracking is not perfect and the base will stop moving in a certain tolerance).

More info about the SMB navigation and control stack here: https://ethz-robotx.github.io/SuperMegaBot/.

### Real Robot

TODO


