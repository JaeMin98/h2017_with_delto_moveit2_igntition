#!/usr/bin/env bash
# This script converts xacro (URDF variant) into SDF for `h2017_description` package

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
XACRO_PATH="$(dirname "${SCRIPT_DIR}")/urdf/h2017.urdf.xacro"
SDF_PATH="$(dirname "${SCRIPT_DIR}")/h2017/model.sdf"

# Arguments for xacro
XACRO_ARGS=(
    name:=h2017
    gripper:=true
    collision_arm:=true
    collision_gripper:=true
    ros2_control:=true
    ros2_control_plugin:=gz
    ros2_control_command_interface:=effort
    gazebo_preserve_fixed_joint:=false
)

# Remove old SDF file
rm "${SDF_PATH}" 2>/dev/null

# Process xacro into URDF, then convert URDF to SDF and edit the SDF to use relative paths for meshes
"${SCRIPT_DIR}/xacro2sdf_direct.bash" "${XACRO_PATH}" "${XACRO_ARGS[@]}" "${@:1}" >"${SDF_PATH}" &&
echo "Created new ${SDF_PATH}"
