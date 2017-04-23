#!/bin/bash

for pid in  `ps -Ao "%p,%a" | grep -v grep | grep -E 'CsConfig|CsScenario|camera_driver|nodelet_manager|CsNetwork|CsDevice|launchScript|realsense_camera|cs_depth_transcode' | cut -f1 -d','`; do
    echo "killing pid: $pid"
    kill -9  $pid
done