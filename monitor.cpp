#include "monitor.h"


 /*直线拟合监测
 * 接收输入点云cloud_base做基础点云，待监测点云cloud_monitor，监测距离阈值Max_distance,用于直线拟合的点的个数FittingPointNumber
 * 遍历待监测点云，计算其与基础点云最近n个点的拟合直线的垂直距离，大于阈值则标记为高程变化的点
 */
int Monitoring_Fittingline(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_base, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_monitor, float Max_distance,int FittingPointNumber)
{
	//设置kdtree
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_base);
	int k = FittingPointNumber;
	std::vector<int>pointIdxNKNSearch(k);
	std::vector<float>pointNKNSquaredDistance(k);

	//设置拟合点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fitting(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_fitting->width = FittingPointNumber;
	cloud_fitting->height = 1;
	cloud_fitting->is_dense = false;
	cloud_fitting->points.resize(cloud_fitting->width * cloud_fitting->height);
	
	double point_to_fittingline_distance;
	int count = 0;
	for (int i = 0; i < cloud_monitor->width; i++)
	{
		kdtree.nearestKSearch(cloud_monitor->points[i], k, pointIdxNKNSearch, pointNKNSquaredDistance);
		for (int j = 0; j < k; j++)
		{
			cloud_fitting->points[j] = cloud_base->points[pointIdxNKNSearch[j]];
		}
		point_to_fittingline_distance = LineFittingReDistance(cloud_fitting, cloud_monitor->points[i], 1);
		if (point_to_fittingline_distance > Max_distance)
		{
			cloud_monitor->points[count] = cloud_monitor->points[i];
			count++;
		}

		
	}
	cloud_monitor->width = count;
	cloud_monitor->points.resize(count);
	return 0;
}