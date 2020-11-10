#pragma once
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include"linefitting.h"
#include<pcl/kdtree/kdtree_flann.h>

int Monitoring_Fittingline(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_base, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_monitor, float Max_distance, int FittingPointNumber);