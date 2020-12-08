#pragma once
#include<iostream>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<pcl/io/pcd_io.h>
#include<pcl/registration/icp.h>
#include<pcl/visualization/pcl_visualizer.h>
#include "txtfile.h"
#include"filters.h"
#include"connected.h"
#include"linefitting.h"
#include"monitor.h"
#include<cmath>

void SingleFileProcess();
void BatchProcessingSeed();
void BatchProcessingNeighbor();