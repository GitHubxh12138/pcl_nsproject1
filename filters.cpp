#include"filters.h"

//
int NextDirectionFilter(pcl::PointCloud<pcl::PointXYZ>& cloud_in, int NextPointNumber,float threshold)
{
	pcl::PointCloud<pcl::PointXYZI> cloud_transfer;//中转点云
	cloud_transfer.width = cloud_in.width;
	cloud_transfer.height = cloud_in.height;
	cloud_transfer.is_dense = cloud_in.is_dense;
	cloud_transfer.points.resize(cloud_transfer.width * cloud_transfer.height);
	
	for (int i = 0; i < cloud_in.width * cloud_transfer.height; i++)
	{
		//将点云值赋给中转点云
		cloud_transfer.points[i].x = cloud_in.points[i].x;
		cloud_transfer.points[i].y = cloud_in.points[i].y;
		cloud_transfer.points[i].z = cloud_in.points[i].z;
		//计算前高差梯度,存放于前点的Intensity
		
		if (i)
		{
			cloud_transfer.points[i-1].intensity= (cloud_in.points[i].z - cloud_in.points[i  - 1].z) / (std::sqrt(std::pow(cloud_in.points[i ].x - cloud_in.points[i - 1].x, 2) + std::pow(cloud_in.points[i ].y - cloud_in.points[i  - 1].y, 2)));
			/*if (i > 1 && std::sqrt(std::pow(cloud_in.points[i].x - cloud_in.points[i - 1].x, 2) + std::pow(cloud_in.points[i].y - cloud_in.points[i - 1].y, 2)) < 0.015)
			{
				cloud_transfer.points[i - 1].intensity = (cloud_in.points[i].z - cloud_in.points[i - 2].z) / (std::sqrt(std::pow(cloud_in.points[i].x - cloud_in.points[i - 2].x, 2) + std::pow(cloud_in.points[i].y - cloud_in.points[i - 2].y, 2)));
			}*/
		}

	}
	int count = 0;
	for (int i = 0; i < cloud_in.width * cloud_transfer.height; i++)
	{
		int choosen=0;
		//int a = 2643;
		//if (i <= a+10&&i>=a-10)
		//{
		//	std::cout << i << " "<<cloud_transfer.points[i].intensity << std::endl;



		//}
		//if(i=3033)

		if (i < cloud_in.width * cloud_transfer.height - 1 - NextPointNumber)
		{
			int judge = 1;
			for (int j = 0; j < NextPointNumber; j++)
			{
				//float a = (cloud_transfer.points[i + j + 1].intensity - cloud_transfer.points[i + j].intensity);
				if (std::abs(cloud_transfer.points[i + j + 1].intensity - cloud_transfer.points[i + j].intensity) > threshold)
					judge =0;
			}
			if (judge)
				choosen = 1;
		}
		if (i > NextPointNumber)
		{
			int judge = 1;
			for (int j = 0; j < NextPointNumber; j++)
			{
				if (std::abs(cloud_transfer.points[i - NextPointNumber+j].intensity - cloud_transfer.points[i - NextPointNumber +j-1].intensity) > threshold)
					judge =0;
			}
			if (judge)
				choosen = 1;
		}
		if (choosen)
		{
			cloud_in.points[count].x = cloud_transfer.points[i].x;
			cloud_in.points[count].y = cloud_transfer.points[i].y;
			cloud_in.points[count].z = cloud_transfer.points[i].z;
			count++;
		}
	}
	cloud_in.width = count;
	cloud_in.points.resize(count);
	return 0;
}


int LineFittingFilter(pcl::PointCloud<pcl::PointXYZ>& cloud_in, int NextPointNumber, float Fitting_Max_Distance,float Filter_Max_distance)
{
	pcl::PointCloud<pcl::PointXYZI> cloud_transfer;//中转点云
	cloud_transfer.width = cloud_in.width;
	cloud_transfer.height = cloud_in.height;
	cloud_transfer.is_dense = cloud_in.is_dense;
	cloud_transfer.points.resize(cloud_transfer.width * cloud_transfer.height);

	
	pcl::PointCloud<pcl::PointXYZ> cloud_linefitting;//拟合直线点云
	cloud_linefitting.width = NextPointNumber;
	cloud_linefitting.height = 1;
	cloud_linefitting.is_dense = cloud_in.is_dense;
	cloud_linefitting.points.resize(cloud_linefitting.width * cloud_linefitting.height);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_linefitting_ptr(new pcl::PointCloud<pcl::PointXYZ>);//指针
	cloud_linefitting_ptr = cloud_linefitting.makeShared();

	for (int i = 0; i < cloud_in.width * cloud_transfer.height; i++)
	{
		//将点云值赋给中转点云
		cloud_transfer.points[i].x = cloud_in.points[i].x;
		cloud_transfer.points[i].y = cloud_in.points[i].y;
		cloud_transfer.points[i].z = cloud_in.points[i].z;

		if (i < cloud_in.width * cloud_transfer.height - NextPointNumber)
		{
			for (int j = 0; j < NextPointNumber; j++)
			{
				cloud_linefitting_ptr->points[j].x = cloud_in.points[i + 1 + j].x;
				cloud_linefitting_ptr->points[j].y = cloud_in.points[i + 1 + j].y;
				cloud_linefitting_ptr->points[j].z = cloud_in.points[i + 1 + j].z;
			}
			cloud_transfer.points[i].intensity = LineFittingReDistance(cloud_linefitting_ptr, cloud_in.points[i], Fitting_Max_Distance);
			
		}
		else
		{
			cloud_transfer.points[i].intensity = 100;
		}

		if (i >= NextPointNumber)
		{
			for (int j = 0; j < NextPointNumber; j++)
			{
				cloud_linefitting_ptr->points[j].y = cloud_in.points[i-NextPointNumber+j].y;
				cloud_linefitting_ptr->points[j].z = cloud_in.points[i-NextPointNumber+j].z;
				cloud_linefitting_ptr->points[j].x = cloud_in.points[i-NextPointNumber+j].x;
			}
			float fcompare = LineFittingReDistance(cloud_linefitting_ptr, cloud_in.points[i], Fitting_Max_Distance);
			if (fcompare < cloud_transfer.points[i].intensity)
			{
				cloud_transfer.points[i].intensity = fcompare;
			}
			
		}
		
		

	}
	int count = 0;
	for (int i = 0; i < cloud_in.width * cloud_transfer.height; i++)
	{
		

		
		if (cloud_transfer.points[i].intensity<Filter_Max_distance)
		{
			cloud_in.points[count].x = cloud_transfer.points[i].x;
			cloud_in.points[count].y = cloud_transfer.points[i].y;
			cloud_in.points[count].z = cloud_transfer.points[i].z;
			count++;
		}
	}
	cloud_in.width = count;
	cloud_in.points.resize(count);
	return 0;
}