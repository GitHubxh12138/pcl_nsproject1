#pragma once
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<cmath>

int NextDirectionFilter(pcl::PointCloud<pcl::PointXYZ>& cloud_in, int NextPointNumber, float threshold);