#include<iostream>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<pcl/io/pcd_io.h>
#include<pcl/registration/icp.h>
#include<pcl/visualization/pcl_visualizer.h>
#include "txtfile.h"
#include"filters.h"
#include"connected.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
#define Detect_Number 4//向后探测的点的个数
#define Threshold 0.5//高程梯度差
#define Connected_Distance 0.18
#define Connected_Number 25


//批量读取文件并处理输出
int BatchProcessingTxtFile(char *PathOfTxtFile, int NumberOfTxtFile, char* FrontName)
{
	PointCloudT::Ptr cloud_in(new PointCloudT);
	PointCloudT::Ptr cloud_out(new PointCloudT);
	PointT cloud_in_shift;//点云整体平移参数
	int cloud_in_shift_exist = 0;
	char FileName[100];
	for (int i = 0; i < NumberOfTxtFile; i++)
	{
    //读txt文件
	std:; sprintf_s(FileName, sizeof(FileName), "%s%s%d%s", PathOfTxtFile, FrontName, i+1, ".txt");
	 ReadTxtFile(FileName, *cloud_in, cloud_in_shift_exist, cloud_in_shift);

	*cloud_out = *cloud_in;//复制点云
	NextDirectionFilter(*cloud_out, Detect_Number, Threshold);//点云高程梯度滤波
	LabelConnectedComponent(*cloud_out, Connected_Distance, Connected_Number);//点云连通分析

	//输出
	sprintf_s(FileName, sizeof(FileName), "%sres %s%d%s", PathOfTxtFile, FrontName, i+1, ".txt");
	WriteTxtFile(FileName, *cloud_out, cloud_in_shift);
	}
	return 0;
}


int main(int argc, char** argv)
{
	//单一文件

	//PointCloudT::Ptr cloud_in1(new PointCloudT);
	//PointCloudT::Ptr cloud_in2(new PointCloudT);
	//PointCloudT::Ptr cloud_out1(new PointCloudT);
	//PointCloudT::Ptr cloud_out2(new PointCloudT);
	//PointT cloud_in_shift = { 0,0,0 };//点云整体平移参数
	//int cloud_in_shift_exist = 0;
	//ReadTxtFile("H:\\first\\1\\cross_21.txt",*cloud_in1, cloud_in_shift_exist, cloud_in_shift);//读txt文件
	//ReadTxtFile("H:\\second\\1\\cross_21.txt", *cloud_in2, cloud_in_shift_exist, cloud_in_shift);
	//*cloud_out1 = *cloud_in1;//复制点云
	//*cloud_out2 = *cloud_in2;

	//
	//
	//pcl::IterativeClosestPoint<PointT, PointT> icp;
	//icp.setMaximumIterations(100);
	//icp.setInputSource(cloud_out2);
	//icp.setInputTarget(cloud_out1);
	//icp.align(*cloud_out2);


	//NextDirectionFilter(*cloud_out1, Detect_Number, Threshold);//点云高程梯度滤波
	//NextDirectionFilter(*cloud_out2, Detect_Number, Threshold);//点云高程梯度滤波
	//WriteTxtFile("一期21高程结果.txt", *cloud_out1, cloud_in_shift);//输出
	//LabelConnectedComponent(*cloud_out1, Connected_Distance, Connected_Number);//点云连通分析
	//LabelConnectedComponent(*cloud_out2, Connected_Distance, Connected_Number);
	//WriteTxtFile("一期21提取结果.txt", *cloud_out1, cloud_in_shift);//输出
	//WriteTxtFile("二期21提取结果.txt", *cloud_out2, cloud_in_shift);//输出

	//批量处理
	char PathOfTxtFile[30];
	int NumberOfTxtFile;
	char FrontName[20];
	std::cout << "请输入文件路径，文件个数，文件头名" << std::endl;
	std::cin >> PathOfTxtFile >> NumberOfTxtFile >> FrontName;
	BatchProcessingTxtFile(PathOfTxtFile, NumberOfTxtFile, FrontName);

////可视化
	//pcl::visualization::PCLVisualizer viewer("ICP demo");

	////创建1个观察点
	//int v1(0);
	//viewer.createViewPort(0.0, 0.0, 1.0, 1.0, v1);


	////定义显示颜色信息
	//float bckgr_gray_level = 0.0;
	//float txt_gray_level = 1.0 - bckgr_gray_level;

	////原始的点云设置为白色
	//pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud_in, (int)255 * txt_gray_level, (int)255 * txt_gray_level, (int)255 * txt_gray_level);
	//viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v1", v1);


	////转换后的点云设置为绿色
	//pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_out_color_h(cloud_out, 20, 180, 20);
	//viewer.addPointCloud(cloud_out, cloud_out_color_h, "cloud_tr_v1", v1);



	////设置背景颜色
	//viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);


	////设置相机的坐标和方向
	//viewer.setCameraPosition(-3, 2, 5,-100,100,-17, 0.2, 0.9, -0.2, 0);


	////显示
	//while (!viewer.wasStopped())
	//{
	//	viewer.spinOnce();


	//}




	

	
	return(0);
}


