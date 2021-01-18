#!/usr/bin/env zsh

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

xacro "$DIR"/cupboard_drawers/cupboard_drawers.urdf.xacro > "$DIR"/cupboard_drawers/cupboard_drawers.urdf
xacro "$DIR"/container/container_no_lid.urdf.xacro > "$DIR"/container/container_no_lid.urdf
xacro "$DIR"/container/lid.urdf.xacro > "$DIR"/container/lid.urdf
xacro "$DIR"/container/container_sliding_lid.urdf.xacro > "$DIR"/container/container_sliding_lid.urdf
xacro "$DIR"/box_panda_hand.urdf.xacro > "$DIR"/box_panda_hand_pb.urdf

xacro "$DIR"/container/container_removable_lid.urdf.xacro > "$DIR"/container/container_removable_lid.urdf
xacro "$DIR"/container/container_sliding_lid_on_table.urdf.xacro > "$DIR"/container/container_sliding_lid_on_table.urdf
xacro "$DIR"/container/container_removable_lid_on_table.urdf.xacro > "$DIR"/container/container_removable_lid_on_table.urdf

FRANKA_DESCR_PATH="$(rospack find franka_description | sed 's./.\\\/.g')"
sed -i "s/package:\/\/franka_description/$FRANKA_DESCR_PATH/g" "$DIR"/box_panda_hand_pb.urdf
