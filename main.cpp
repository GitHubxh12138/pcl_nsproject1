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
#include"fileprocess.h"



int main(int argc, char** argv)
{
	//SingleFileProcess();//单一文件处理
	//BatchProcessingSeed();//批量处理文件种子法
	BatchProcessingNeighbor();//批量处理邻近法

	

	

	
	return(0);
}


