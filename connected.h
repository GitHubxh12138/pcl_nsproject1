#pragma once
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<cmath>
#include<pcl/kdtree/kdtree_flann.h>
#include"txtfile.h"

void LabelConnectedComponent(pcl::PointCloud<pcl::PointXYZ>& cloud_in, float max_distance, int minimum_number);
int Connnected_Select(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_base, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_data, float distance);
void NeighborLineAnalysisoneline(pcl::PointCloud<pcl::PointXYZ>& cloud_in, float max_distance, int minimum_number, int nrline, int totalnrline);
void NeighborLineAnalysistwoline(pcl::PointCloud<pcl::PointXYZ>& cloud_in, float max_distance, int minimum_number, int nrline, int totalnrline);