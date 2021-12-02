# moma_node_manager
Installation and usage instructions for node manager.
## Installation
Install from source on noetic, since the default Ubuntu package does not work:
  - `cd catkin_ws/src`
  - `vcs import --recursive --input moma/moma_node_manager.repos`
  - (Optional) `sudo apt install python3-rosdep python3-grpc-tools`
  - `sudo rosdep init`
  - `rosdep update`
  - `rosdep install -i --as-root pip:false --reinstall --from-paths multimaster_fkie`
  - `catkin build fkie_node_manager`(`_daemon`)` fkie_multimaster`

## Configuration
Configure all PCs you want to manage using node manager:
  - Comment out the following snippet in `~/.bashrc` on the PCs managed by node manager:
    ```
    # If not running interactively, don't do anything
    #case $- in
    #    *i*) ;;
    #      *) return;;
    #esac
    ```

## ToDo
  - Issue for daemon:
    https://github.com/fkie/multimaster_fkie/issues/161