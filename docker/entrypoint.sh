#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=========================================="
echo " Barracuda DVL Workspace Ready! "
echo "=========================================="

exec "$@"