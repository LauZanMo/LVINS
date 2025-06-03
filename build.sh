#!/bin/bash
set -e

project_dir=$(dirname "$(realpath "$0")")

echo "Installing prerequisites..."

cd "$project_dir"/../
vcs-import < LVINS/dependencies.yaml
pip install -r LVINS/requirements.txt
cd ..
rosdep install --from-paths src --ignore-src -r -y

echo "Building ROS 2 workspace..."

cp src/LVINS/colcon.meta .
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON