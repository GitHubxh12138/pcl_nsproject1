#pragma once
#include<iostream>
#include<fstream>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include <iomanip>
#include<stdio.h>
#include<stdlib.h>
#include<sstream>

int getNumberOfEdges(const char* file_name);
int getCmdRes(const char* sc);
void ReadTxtFile(const char* file_name, pcl::PointCloud<pcl::PointXYZ>& cloud);
int WriteTxtFile(const char* file_name, pcl::PointCloud<pcl::PointXYZ>& cloud);
int OnlyWriteTxtFile(const char* file_name, pcl::PointCloud<pcl::PointXYZ>& cloud);