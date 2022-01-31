#! /bin/bash
outpath="$1"

# Other variables
now="$(date +"%F-%H-%M-%S")"
today="$(date +"%F")"

# List of useful colors
COLOR_RESET="\033[0m"
COLOR_WARN="\033[0;33m"

# Check outputpath
if [ "${outpath}" == "" ]; then
  outpath="${HOME}/bags/${today}"
fi

if [ ! -d "${outpath}" ]; then
  mkdir -p "${outpath}"
fi

# Record
rosparam dump ${outpath}/${now}.yaml
rosbag record --repeat-latched --output-name="${outpath}/${now}_valve_perception" \
/tf \
/tf_static \
/camera_info \
/hand_eye/color/image_raw_throttle \
/hand_eye/depth/image_rect_raw_throttle \
/hand_eye/depth/color/points_throttle \
/joint_states
