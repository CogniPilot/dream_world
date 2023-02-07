#!/bin/bash
set -e
set -x
cd /workdir/gazebo
colcon build --cmake-args -DBUILD_TESTING=OFF --merge-install
