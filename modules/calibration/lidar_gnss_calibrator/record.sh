#!/usr/bin/env bash

source /apollo/scripts/apollo_base.sh
rosbag record -O multi_lidar_gnss.bag /apollo/sensor/velodyne16/PointCloud2 /apollo/sensor/velodyne64/PointCloud2 /apollo/sensor/gnss/odometry
mkdir -p /apollo/data/bag/calibration/
mv multi_lidar_gnss.bag /apollo/data/bag/calibration/
