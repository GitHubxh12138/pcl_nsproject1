#pragma once
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<cmath>
#include"linefitting.h"

int NextDirectionFilter(pcl::PointCloud<pcl::PointXYZ>& cloud_in, int NextPointNumber, float threshold);
int LineFittingFilter(pcl::PointCloud<pcl::PointXYZ>& cloud_in, int NextPointNumber, float Fitting_Max_Distance, float Filter_Max_distance);