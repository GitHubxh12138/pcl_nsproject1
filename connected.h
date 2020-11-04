#pragma once
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<cmath>


void LabelConnectedComponent(pcl::PointCloud<pcl::PointXYZ>& cloud_in, float max_distance, int minimum_number);