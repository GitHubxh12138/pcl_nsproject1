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
#define Detect_Number 4//���̽��ĵ�ĸ���
#define Threshold 0.5//�߳��ݶȲ�
#define Connected_Distance 0.18
#define Connected_Number 25


//������ȡ�ļ����������
int BatchProcessingTxtFile(char *PathOfTxtFile, int NumberOfTxtFile, char* FrontName)
{
	PointCloudT::Ptr cloud_in(new PointCloudT);
	PointCloudT::Ptr cloud_out(new PointCloudT);
	PointT cloud_in_shift;//��������ƽ�Ʋ���
	int cloud_in_shift_exist = 0;
	char FileName[100];
	for (int i = 0; i < NumberOfTxtFile; i++)
	{
    //��txt�ļ�
	std:; sprintf_s(FileName, sizeof(FileName), "%s%s%d%s", PathOfTxtFile, FrontName, i+1, ".txt");
	 ReadTxtFile(FileName, *cloud_in, cloud_in_shift_exist, cloud_in_shift);

	*cloud_out = *cloud_in;//���Ƶ���
	NextDirectionFilter(*cloud_out, Detect_Number, Threshold);//���Ƹ߳��ݶ��˲�
	LabelConnectedComponent(*cloud_out, Connected_Distance, Connected_Number);//������ͨ����

	//���
	sprintf_s(FileName, sizeof(FileName), "%sres %s%d%s", PathOfTxtFile, FrontName, i+1, ".txt");
	WriteTxtFile(FileName, *cloud_out, cloud_in_shift);
	}
	return 0;
}


int main(int argc, char** argv)
{
	//��һ�ļ�

	//PointCloudT::Ptr cloud_in1(new PointCloudT);
	//PointCloudT::Ptr cloud_in2(new PointCloudT);
	//PointCloudT::Ptr cloud_out1(new PointCloudT);
	//PointCloudT::Ptr cloud_out2(new PointCloudT);
	//PointT cloud_in_shift = { 0,0,0 };//��������ƽ�Ʋ���
	//int cloud_in_shift_exist = 0;
	//ReadTxtFile("H:\\first\\1\\cross_21.txt",*cloud_in1, cloud_in_shift_exist, cloud_in_shift);//��txt�ļ�
	//ReadTxtFile("H:\\second\\1\\cross_21.txt", *cloud_in2, cloud_in_shift_exist, cloud_in_shift);
	//*cloud_out1 = *cloud_in1;//���Ƶ���
	//*cloud_out2 = *cloud_in2;

	//
	//
	//pcl::IterativeClosestPoint<PointT, PointT> icp;
	//icp.setMaximumIterations(100);
	//icp.setInputSource(cloud_out2);
	//icp.setInputTarget(cloud_out1);
	//icp.align(*cloud_out2);


	//NextDirectionFilter(*cloud_out1, Detect_Number, Threshold);//���Ƹ߳��ݶ��˲�
	//NextDirectionFilter(*cloud_out2, Detect_Number, Threshold);//���Ƹ߳��ݶ��˲�
	//WriteTxtFile("һ��21�߳̽��.txt", *cloud_out1, cloud_in_shift);//���
	//LabelConnectedComponent(*cloud_out1, Connected_Distance, Connected_Number);//������ͨ����
	//LabelConnectedComponent(*cloud_out2, Connected_Distance, Connected_Number);
	//WriteTxtFile("һ��21��ȡ���.txt", *cloud_out1, cloud_in_shift);//���
	//WriteTxtFile("����21��ȡ���.txt", *cloud_out2, cloud_in_shift);//���

	//��������
	char PathOfTxtFile[30];
	int NumberOfTxtFile;
	char FrontName[20];
	std::cout << "�������ļ�·�����ļ��������ļ�ͷ��" << std::endl;
	std::cin >> PathOfTxtFile >> NumberOfTxtFile >> FrontName;
	BatchProcessingTxtFile(PathOfTxtFile, NumberOfTxtFile, FrontName);

////���ӻ�
	//pcl::visualization::PCLVisualizer viewer("ICP demo");

	////����1���۲��
	//int v1(0);
	//viewer.createViewPort(0.0, 0.0, 1.0, 1.0, v1);


	////������ʾ��ɫ��Ϣ
	//float bckgr_gray_level = 0.0;
	//float txt_gray_level = 1.0 - bckgr_gray_level;

	////ԭʼ�ĵ�������Ϊ��ɫ
	//pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud_in, (int)255 * txt_gray_level, (int)255 * txt_gray_level, (int)255 * txt_gray_level);
	//viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v1", v1);


	////ת����ĵ�������Ϊ��ɫ
	//pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_out_color_h(cloud_out, 20, 180, 20);
	//viewer.addPointCloud(cloud_out, cloud_out_color_h, "cloud_tr_v1", v1);



	////���ñ�����ɫ
	//viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);


	////�������������ͷ���
	//viewer.setCameraPosition(-3, 2, 5,-100,100,-17, 0.2, 0.9, -0.2, 0);


	////��ʾ
	//while (!viewer.wasStopped())
	//{
	//	viewer.spinOnce();


	//}




	

	
	return(0);
}


