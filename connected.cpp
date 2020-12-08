#include"connected.h"

 /*点云连通分析
 * 读入输入点云cloud_in，联通分析点距离阈值max_distance，联通分析连通域内点最少数量minimun_number
 * 
 */
void LabelConnectedComponent(pcl::PointCloud<pcl::PointXYZ>& cloud_in, float max_distance, int minimum_number)
{
	pcl::PointCloud<pcl::PointXYZI> cloud_transfer;//中转点云
	cloud_transfer.width = cloud_in.width;
	cloud_transfer.height = cloud_in.height;
	cloud_transfer.is_dense = cloud_in.is_dense;
	cloud_transfer.points.resize(cloud_transfer.width * cloud_transfer.height);

	int Number = 1;
	int Count = 0;
	for (int i = 0; i < cloud_transfer.width * cloud_transfer.height; i++)
	{
		//将点云值赋给中转点云
		cloud_transfer.points[i].x = cloud_in.points[i].x;
		cloud_transfer.points[i].y = cloud_in.points[i].y;
		cloud_transfer.points[i].z = cloud_in.points[i].z;


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


//连通分析以及相邻线判断，只需要一根线上有邻近点
void NeighborLineAnalysisoneline(pcl::PointCloud<pcl::PointXYZ>& cloud_in, float max_distance, int minimum_number,int nrline,int totalnrline)
{
	pcl::PointCloud<pcl::PointXYZI> cloud_transfer;//中转点云
	cloud_transfer.width = cloud_in.width;
	cloud_transfer.height = cloud_in.height;
	cloud_transfer.is_dense = cloud_in.is_dense;
	cloud_transfer.points.resize(cloud_transfer.width * cloud_transfer.height);

	int Number = 1;
	int Count = 0;
	for (int i = 0; i < cloud_transfer.width * cloud_transfer.height; i++)
	{
		//将点云值赋给中转点云
		cloud_transfer.points[i].x = cloud_in.points[i].x;
		cloud_transfer.points[i].y = cloud_in.points[i].y;
		cloud_transfer.points[i].z = cloud_in.points[i].z;


		if (i)
		{
			cloud_transfer.points[i - 1].intensity = std::sqrt(std::pow(cloud_transfer.points[i].x - cloud_transfer.points[i - 1].x, 2) + std::pow(cloud_transfer.points[i].y - cloud_transfer.points[i - 1].y, 2) + std::pow(cloud_transfer.points[i].z - cloud_transfer.points[i - 1].z, 2));

			if (cloud_transfer.points[i - 1].intensity < max_distance)
			{
				Number++;
			}
			else
			{
				if (Number >= minimum_number)
				{
					float avex=0, avey=0, avez=0;
					for (int j = 0; j < Number; j++)
					{
						avex += cloud_transfer.points[i - Number + j].x;
						avey += cloud_transfer.points[i - Number + j].y;
						avez += cloud_transfer.points[i - Number + j].z;
					}
					avex/=Number;
					avey/=Number;
					avez/=Number;
					pcl::PointXYZ avep = { avex,avey,avez };
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_neighbor(new pcl::PointCloud<pcl::PointXYZ>);
					char FileName[100];

					int judge = 0;
					if (nrline - 1 > 0)
					{
						sprintf(FileName, "./second/1/filterresult%d.txt",nrline-1);
						ReadTxtFile(FileName, *cloud_neighbor);

						pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
						kdtree.setInputCloud(cloud_neighbor);
						std::vector<int>pointIdxNKNSearch;
						std::vector<float>pointNKNSquaredDistance;
						kdtree.nearestKSearch(avep, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
						if (pointNKNSquaredDistance[0] < std::pow(0.6, 2))
						{
							judge = 1;
						}
					}
					if (nrline + 1 <= totalnrline)
					{
						sprintf(FileName, "./second/1/filterresult%d.txt", nrline + 1);
						ReadTxtFile(FileName, *cloud_neighbor);

						pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
						kdtree.setInputCloud(cloud_neighbor);
						std::vector<int>pointIdxNKNSearch;
						std::vector<float>pointNKNSquaredDistance;
						kdtree.nearestKSearch(avep, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
						if (pointNKNSquaredDistance[0] < std::pow(0.6, 2))
						{
							judge = 1;
						}
					}
					if (judge)
					{
						for (int j = 0; j < Number; j++)
						{
							cloud_in.points[Count].x = cloud_transfer.points[i - Number + j].x;
							cloud_in.points[Count].y = cloud_transfer.points[i - Number + j].y;
							cloud_in.points[Count].z = cloud_transfer.points[i - Number + j].z;
							Count++;
						}
					}




					
				}
				Number = 1;
			}
		}

	}
	cloud_in.width = Count;
	cloud_in.points.resize(Count);
}


//连通分析以及相邻线判断，需要两根线上有邻近点
void NeighborLineAnalysistwoline(pcl::PointCloud<pcl::PointXYZ>& cloud_in, float max_distance, int minimum_number, int nrline, int totalnrline)
{
	pcl::PointCloud<pcl::PointXYZI> cloud_transfer;//中转点云
	cloud_transfer.width = cloud_in.width;
	cloud_transfer.height = cloud_in.height;
	cloud_transfer.is_dense = cloud_in.is_dense;
	cloud_transfer.points.resize(cloud_transfer.width * cloud_transfer.height);

	int Number = 1;
	int Count = 0;
	for (int i = 0; i < cloud_transfer.width * cloud_transfer.height; i++)
	{
		//将点云值赋给中转点云
		cloud_transfer.points[i].x = cloud_in.points[i].x;
		cloud_transfer.points[i].y = cloud_in.points[i].y;
		cloud_transfer.points[i].z = cloud_in.points[i].z;


		if (i)
		{
			cloud_transfer.points[i - 1].intensity = std::sqrt(std::pow(cloud_transfer.points[i].x - cloud_transfer.points[i - 1].x, 2) + std::pow(cloud_transfer.points[i].y - cloud_transfer.points[i - 1].y, 2) + std::pow(cloud_transfer.points[i].z - cloud_transfer.points[i - 1].z, 2));

			if (cloud_transfer.points[i - 1].intensity < max_distance)
			{
				Number++;
			}
			else
			{
				if (Number >= minimum_number)
				{
					float avex = 0, avey = 0, avez = 0;
					for (int j = 0; j < Number; j++)
					{
						avex += cloud_transfer.points[i - Number + j].x;
						avey += cloud_transfer.points[i - Number + j].y;
						avez += cloud_transfer.points[i - Number + j].z;
					}
					avex /= Number;
					avey /= Number;
					avez /= Number;
					pcl::PointXYZ avep = { avex,avey,avez };
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_neighbor(new pcl::PointCloud<pcl::PointXYZ>);
					char FileName[100];

					int judge = 0;
					if (nrline - 1 > 0)
					{
						sprintf(FileName, "./second/1/filterresult%d.txt", nrline - 1);
						ReadTxtFile(FileName, *cloud_neighbor);

						pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
						kdtree.setInputCloud(cloud_neighbor);
						std::vector<int>pointIdxNKNSearch;
						std::vector<float>pointNKNSquaredDistance;
						kdtree.nearestKSearch(avep, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
						if (pointNKNSquaredDistance[0] < std::pow(0.6, 2))
						{
							judge ++;
						}
					}
					if (nrline + 1 <= totalnrline)
					{
						sprintf(FileName, "./second/1/filterresult%d.txt", nrline + 1);
						ReadTxtFile(FileName, *cloud_neighbor);

						pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
						kdtree.setInputCloud(cloud_neighbor);
						std::vector<int>pointIdxNKNSearch;
						std::vector<float>pointNKNSquaredDistance;
						kdtree.nearestKSearch(avep, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
						if (pointNKNSquaredDistance[0] < std::pow(0.6, 2))
						{
							judge ++;
						}
					}
					if (judge==2)
					{
						for (int j = 0; j < Number; j++)
						{
							cloud_in.points[Count].x = cloud_transfer.points[i - Number + j].x;
							cloud_in.points[Count].y = cloud_transfer.points[i - Number + j].y;
							cloud_in.points[Count].z = cloud_transfer.points[i - Number + j].z;
							Count++;
						}
					}





				}
				Number = 1;
			}
		}

	}
	cloud_in.width = Count;
	cloud_in.points.resize(Count);
}

 /*临近点选择
 * 接收输入点云cloud_base作为选择基础，cloud_data为待选择点云，distace为选择距离阈值
 * 遍历待选择点云，计算其与基础点云最近点的水平距离，若小于距离阈值则保留
 */
int Connnected_Select(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_base, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_data, float distance)
{
	//设置kdtree
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_base);
	int k = 1;
	std::vector<int>pointIdxNKNSearch(k);
	std::vector<float>pointNKNSquaredDistance(k);


	
	int count = 0;
	double xydistance;
	//提取近点
	for (int i = 0; i < cloud_data->width; i++)
	{
		kdtree.nearestKSearch(cloud_data->points[i], k, pointIdxNKNSearch, pointNKNSquaredDistance);
		xydistance = std::sqrt(pow(cloud_data->points[i].x - cloud_base->points[pointIdxNKNSearch[0]].x ,2) + pow(cloud_data->points[i].y - cloud_base->points[pointIdxNKNSearch[0]].y, 2));

		if (xydistance < distance )
		{
			cloud_data->points[count].x = cloud_data->points[i].x;
			cloud_data->points[count].y = cloud_data->points[i].y;
			cloud_data->points[count].z = cloud_data->points[i].z;
			count++;
		}
	}
	cloud_data->width = count;
	cloud_data->points.resize(count);
	return 0;
}