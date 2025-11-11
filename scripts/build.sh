#!/usr/bin/env bash

# Usage: ./scripts/build.sh [DOCKER_CI_DIR]

set -e
set -o pipefail

DOCKER_CI_DIR=$1

sudo apt update
sudo apt install -y libboost-log-dev

$DOCKER_CI_DIR/scripts/build.sh sonia_common_ros2

cd depth_port_manager

source /build/sonia_common_ros2/INSTALL_BASE/setup.sh

colcon build --cmake-force-configure --install INSTALL_BASE

set +e

colcon test --return-code-on-test-failure --install INSTALL_BASE

TEST_STATUS=$?

set -e

if [[ $TEST_STATUS != 0 ]]
then
	colcon test-result
fi
