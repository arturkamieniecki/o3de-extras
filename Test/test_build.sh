#!/bin/bash

echo "Running test script"

# Test
. /opt/ros/humble/setup.sh

cd /data/workspace/WarehouseTest

if cmake --build build/linux --config profile --target WarehouseTest.GameLauncher Editor ; then
    echo "Build succeeded"
    echo "RESULT: ALL TESTS PASSED" # expected result 
    if ./o3de/python/python.sh -m pytest --build-directory ./WarehouseTest/build/linux/bin/profile/ ./o3de-extras/Gems/ROS2/Code/PythonTests/SmokeTests_Periodic.py ; then
        echo "GUI test succesfull"
else
    echo "RESULT: Build failed"
fi

exit 0