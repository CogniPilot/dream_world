#!/bin/bash
set -e
set -x
sudo apt update
sudo apt-get install -y python3-vcstool python3-colcon-common-extensions
mkdir -p /workdir/gazebo/src
cd /workdir/gazebo/src
wget https://raw.githubusercontent.com/rudislabs/gazebodistro/cognipilot/collection-garden.yaml -O collection-garden.yaml
vcs import < collection-garden.yaml

#sudo apt -y install \
#  $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ')

cd /workdir/gazebo
colcon graph
colcon build --cmake-args -DBUILD_TESTING=OFF --merge-install
