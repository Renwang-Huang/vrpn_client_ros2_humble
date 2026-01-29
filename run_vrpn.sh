#!/bin/bash

echo "--------------------------------"
echo "launching vrpn_listener..."
echo "--------------------------------"
ros2 launch vrpn_listener vrpn_client.launch &

VRPN_PID=$!
sleep 2

echo "--------------------------------"
echo "launching vicon_to_mavros.py..."
echo "--------------------------------"

# python3 ~/vrpn_client_ros2/vicon_to_mavros.py
python3 ~/vrpn_client_ros2/vprn_to_px4.py

kill $VRPN_PID
