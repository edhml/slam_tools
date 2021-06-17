# Copyright 2021 The LYDS Authors
#

set -o errexit
set -o verbose

VERSION="conditional_build_of_builtin_types"
# test: a0585942a9e1a7255aa7c43452fd5fb2a8b869a4

sudo apt update

# Build and install g2o library
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
git checkout -b ${VERSION} origin/${VERSION}

[ ! -d "/usr/include/eigen3" ] && ./script/install-deps-linux.sh

mkdir build && cd $_
cmake ..
make -j`nproc`
sudo make install
sudo ldconfig

cd ../../
rm -rf g2o

# Install necessary libraries
sudo apt install -y \
        libpcap-dev \
        libyaml-cpp-dev

sudo apt install -y \
	ros-${ROS_DISTRO}-pcl-conversions \
	ros-${ROS_DISTRO}-diagnostic-updater \
	ros-${ROS_DISTRO}-angles \
	ros-${ROS_DISTRO}-slam-toolbox \
	ros-${ROS_DISTRO}-rviz2

