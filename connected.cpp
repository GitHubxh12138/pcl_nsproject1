#include"connected.h"


void LabelConnectedComponent(pcl::PointCloud<pcl::PointXYZ>& cloud_in, float max_distance, int minimum_number)
{
	pcl::PointCloud<pcl::PointXYZI> cloud_transfer;//��ת����
	cloud_transfer.width = cloud_in.width;
	cloud_transfer.height = cloud_in.height;
	cloud_transfer.is_dense = cloud_in.is_dense;
	cloud_transfer.points.resize(cloud_transfer.width * cloud_transfer.height);

	int Number = 1;
	int Count = 0;
	for (int i = 0; i < cloud_transfer.width * cloud_transfer.height; i++)
	{
		//������ֵ������ת����
		cloud_transfer.points[i].x = cloud_in.points[i].x;
		cloud_transfer.points[i].y = cloud_in.points[i].y;
		cloud_transfer.points[i].z = cloud_in.points[i].z;
		//����ǰ�߲��ݶ�,�����ǰ���Intensity

		if (i)
		{
			cloud_transfer.points[i - 1].intensity = std::sqrt(std::pow(cloud_transfer.points[i].x-cloud_transfer.points[i-1].x,2)+ std::pow(cloud_transfer.points[i].y - cloud_transfer.points[i - 1].y, 2) + std::pow(cloud_transfer.points[i].z - cloud_transfer.points[i - 1].z, 2)) ;

			if (cloud_transfer.points[i - 1].intensity < max_distance)
			{
				Number++;
			}
			else
			{
				if (Number >= minimum_number)
				{
					for (int j = 0; j < Number; j++)
					{
						cloud_in.points[Count].x = cloud_transfer.points[i - Number + j].x;
						cloud_in.points[Count].y = cloud_transfer.points[i - Number + j].y;
						cloud_in.points[Count].z = cloud_transfer.points[i - Number + j].z;
						Count++;
					}
				}
				Number = 1;
			}
		}
		
	}
	cloud_in.width = Count;
	cloud_in.points.resize(Count);
}