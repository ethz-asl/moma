# Mobile Manipulation Platform Guide

This repository contains software to run on the Ridgeback mobile platform.

If everything is already set up (should normally be the case), follow the steps in the section [Creating Custom Project Docker Image](#creating-custom-project-docker-image) to get started with your project.

If a basic Ubuntu is already installed and set up on the Ridgeback's onboard computer, but the MobMi Base Docker image needs to be adjusted, follow the steps in the section [Creating New Version of MobMi Base Image](#creating-custom-project-docker-image).

If you want to start from scratch and set up the onboard computer of the platform, follow the steps in the section *Setup of Host OS*.

## Setting up the Ridgeback PC with Ansible

Run the following commands:

```bash
ansible-playbook playbooks/setup_ros.yaml --ask-pass --ask-become-pass -i playbooks/hosts
ansible-playbook playbooks/setup_ridgeback_node.yaml --ask-pass --ask-become-pass -i playbooks/hosts
```

The Docker image with the ridgeback base container still needs to be uploaded and imported manually. For this, see section "Creating New Version of MobMi Base Image".

## Creating Custom Project Docker Image

TODO

## Creating New Version of MobMi Base Image

If the base image docker file `<repo-root>/ridgeback_docker/Dockerfile` was modified, the following steps need to be followed to build it and to install it on the platform.

```bash
cd <repo-root>/ridgeback_docker
```

Build image:

```bash
./build_container.sh
```

Copy to platform:

```bash
scp ../image.zip asl-admin@192.168.131.1:images/image.zip
```

Log into the platform via SSH:

```bash
ssh asl-admin@192.168.131.1
```

Import image:

```bash
docker load -i image.zip
```

Afterwards, the image could be run in a terminal using the following command. However, usually the systemd service will handle this for us (how to set this up is described below).

```bash
/usr/bin/docker run --rm --name asl_base_container --network host -v /dev/input:/dev/input --privileged mobmi
```

The option `--privileged` is required to allow access to USB devices, e.g. the Primesense camera.

## Setup of Host OS

Starting from a clean installation of a host OS (e.g. Ubuntu 18.04), these are the steps to be performed to set up the base node which provides interfaces to the Ridgeback platform and the YuMi.

The user that is used for everything is assumed to be `asl-admin` for this documentation.

### Set up PS4 Controller

Connect PS4 controller through host OS Bluetooth settings (e.g. on Ubuntu through the Bluetooth system settings). This ensures automatic reconnect.

### Install Docker

According to the instructions: [https://docs.docker.com/install/linux/docker-ce/ubuntu/](https://docs.docker.com/install/linux/docker-ce/ubuntu/)

### Set up CAN service

The CAN service initializes the `can0` network interface that is used to communicate with the Ridgeback's MCU, which controls the motors.

1. Copy the file `<repo-root>/mobmi-base-img/srv/asl-can-bus.service` to the host machine under `/etc/systemd/system/`. Set correct owner with `sudo chown root:root /etc/systemd/system/asl-can-bus.service`.
2. Copy the file `<repo-root>/mobmi-base-img/srv/can_start.bash` to the host machine under `/home/asl-admin/scripts/can_start.bash`. This path must coincide with what is specified in the unit file `asl-can-bus.service` from the previous step.
3. Enable the service using the following two commands: `sudo systemctl daemon-reload` and `sudo systemctl enable asl-can-bus.service`.

After the next reboot, the CAN service should be launched automatically. This can be verified by checking if the device `/dev/ttycan0` exists and whether the network interface `can0` exists (the latter using the command `ip addr`).

### Import Docker base container image

To do this, follow the steps outlined in the Section *Creating New Version of MobMi Base Image* above.

### Set up Docker base container service

This service runs the Docker container which in turn runs the base ROS nodes to interface the Ridgeback platform and the YuMi.

1. Copy the file `<repo-root>/mobmi-base-img/srv/asl-base-container.service` to the host machine under `/etc/systemd/system/`. Set correct owner with `sudo chown root:root /etc/systemd/system/asl-base-container.service`.
2. Enable the service using the following two commands: `sudo systemctl daemon-reload` and `sudo systemctl enable asl-base-container.service`.

### Disable System Services

To save resources, disable services that are not required for operation.

- `sudo systemctl set-default multi-user.target` to disable the GUI.

## Other Information

### Startup Order

- `ros.launch` launches
  - `accessories.launch`
  - `base.launch`

### Startup Order in old setup

`/etc/profile.d/clearpath-ros-environment.sh` sources `/etc/ros/setup.bash`, which loads the ros environment and defines some environment variables (`export RIDGEBACK_PS4=1`, `export RIDGEBACK_FRONT_HOKUYO_LASER=1`, ...).

`/etc/init/ros.conf` defines a Linux service which runs `/usr/sbin/ros-start`.

The latter:

- Sources again the ROS environment
- Creates a directory for logging
- Exports `ROS_HOSTNAME` and `ROS_MASTER_URI`
- Combines various xacro files into a single URDF
- Combines calls to all launch files in `/etc/ros/indigo/ros.d` into a single launch file.
- Starts the combined launch file in the background.
- Saves a PID file with the PID of the corresponding process.
