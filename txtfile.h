#pragma once
#include<iostream>
#include<fstream>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include <iomanip>

int getNumberOfEdges(const char* file_name);
void ReadTxtFile(const char* file_name, pcl::PointCloud<pcl::PointXYZ>& cloud, int& cloud_in_shift_exist, pcl::PointXYZ& cloud_in_shift);
int WriteTxtFile(const char* file_name, pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointXYZ cloud_in_shift);