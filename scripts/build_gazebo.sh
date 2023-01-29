#!/bin/bash
set -e
set -x

cd src
sudo apt -y install \
	$(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ')

cd ..
colcon build --cmake-args -DBUILD_TESTING=OFF --merge-install
