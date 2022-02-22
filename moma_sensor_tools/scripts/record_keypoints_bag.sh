#! /bin/bash
outpath="$1"
name="${2:-valve_perception}"
mode="${3:-default}"

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
echo "Recording will be saved with the name ${name}"
rosparam dump ${outpath}/${now}.yaml
if [ $3 = "full" ]; then
  echo "Recording full set of topics (might become huge!)"
  rosbag record --repeat-latched --output-name="${outpath}/${now}_${name}" \
    /tf \
    /tf_static \
    /camera_info \
    /hand_eye/color/image_raw_throttle \
    /hand_eye/depth/image_rect_raw_throttle \
    /hand_eye/depth/color/points_throttle \
    /joint_states
else
  echo "Recording default set of topics"
  rosbag record --repeat-latched --output-name="${outpath}/${now}_${name}" \
    /tf \
    /tf_static \
    /camera_info \
    /hand_eye/color/image_raw_throttle \
    /joint_states
fi
