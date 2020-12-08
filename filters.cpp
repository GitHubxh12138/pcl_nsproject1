#include"filters.h"

/*两点斜率梯度滤波
* 接收进行滤波的点云cloud_in，判断是否为噪声的点探测个数ntnextp，斜率梯度阈值threshold
* 计算点N和点N+1的斜率并存放于点N中，向前探测，计算某点前n个点每相邻两点之间斜率梯度只差，全小于阈值则保留，同理向后探测。
*/
int NextDirectionFilter(pcl::PointCloud<pcl::PointXYZ>& cloud_in, int nrnextp,float threshold)
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

	//向前探测，计算某点前n个点每相邻两点之间斜率梯度只差，全小于阈值则保留，同理向后探测
	int count = 0;
	for (int i = 0; i < cloud_in.width * cloud_transfer.height; i++)
	{
		int choosen=0;
		//int a = 2643;
		//if (i <= a+10&&i>=a-10)
		//{
		//	std::cout << i << " "<<cloud_transfer.points[i].intensity << std::endl;



		//}


		if (i < cloud_in.width * cloud_transfer.height - 1 - nrnextp)
		{
			int judge = 1;
			for (int j = 0; j < nrnextp; j++)
			{
				//float a = (cloud_transfer.points[i + j + 1].intensity - cloud_transfer.points[i + j].intensity);
				if (std::abs(cloud_transfer.points[i + j + 1].intensity - cloud_transfer.points[i + j].intensity) > threshold)
					judge =0;
			}
			if (judge)
				choosen = 1;
		}
		if (i > nrnextp)
		{
			int judge = 1;
			for (int j = 0; j < nrnextp; j++)
			{
				if (std::abs(cloud_transfer.points[i - nrnextp+j].intensity - cloud_transfer.points[i - nrnextp +j-1].intensity) > threshold)
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
 /*点到拟合直线距离滤波
 * 接收进行滤波的点云cloud_in，拟合直线点的个数ntnextp，拟合直线的最大距离Fitting_Max_Distance，点到拟合直线的距离阈值Filter_Max_distance
 * 计算点到拟合直线的距离，距离小于阈值则保留
 */
int LineFittingFilter(pcl::PointCloud<pcl::PointXYZ>& cloud_in, int nrnextp, float Fitting_Max_Distance,float Filter_Max_distance)
{
	pcl::PointCloud<pcl::PointXYZI> cloud_transfer;//中转点云
	cloud_transfer.width = cloud_in.width;
	cloud_transfer.height = cloud_in.height;
	cloud_transfer.is_dense = cloud_in.is_dense;
	cloud_transfer.points.resize(cloud_transfer.width * cloud_transfer.height);

	
	pcl::PointCloud<pcl::PointXYZ> cloud_linefitting;//拟合直线点云
	cloud_linefitting.width = nrnextp;
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

		if (i < cloud_in.width * cloud_transfer.height - nrnextp)
		{
			for (int j = 0; j < nrnextp; j++)
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

		if (i >= nrnextp)
		{
			for (int j = 0; j < nrnextp; j++)
			{
				cloud_linefitting_ptr->points[j].y = cloud_in.points[i-nrnextp+j].y;
				cloud_linefitting_ptr->points[j].z = cloud_in.points[i-nrnextp+j].z;
				cloud_linefitting_ptr->points[j].x = cloud_in.points[i-nrnextp+j].x;
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

/*夹角滤波
* */
int AngleFilter(pcl::PointCloud<pcl::PointXYZ>& cloud_in, int nrnextp, float threshold)
{
	int nrp = cloud_in.width * cloud_in.height;
	double *cos_angle_array=new double[nrp]();
	int* choose_array = new int[nrp]();

	double x1, y1, z1, x2, y2, z2;
	for (int i = 0; i < nrp; i++)
	{
		
		if (i&&i< nrp-1)
		{
			x1 = cloud_in.points[i - 1].x - cloud_in.points[i].x;
			y1 = cloud_in.points[i - 1].y - cloud_in.points[i].y;
			z1 = cloud_in.points[i - 1].z - cloud_in.points[i].z;
			x2 = cloud_in.points[i + 1].x - cloud_in.points[i].x;
			y2 = cloud_in.points[i + 1].y - cloud_in.points[i].y;
			z2 = cloud_in.points[i + 1].z - cloud_in.points[i].z;

			
			cos_angle_array[i]= (x1 * x2 + y1 * y2 + z1 * z2) / (std::sqrt(std::pow(x1, 2) + std::pow(y1, 2) + std::pow(z1, 2)) * sqrt(pow(x2, 2) + pow(y2, 2) + pow(z2, 2)));
			
		}
		

	}

	int count = 0;
	for (int i = 0; i < nrp; i++)
	{
		int choosen = 0;
		

		if (i < nrp - 1 - nrnextp)
		{
			int judge = 1;
			for (int j = 0; j < nrnextp; j++)
			{
				//float a = (cloud_transfer.points[i + j + 1].intensity - cloud_transfer.points[i + j].intensity);
				if (cos_angle_array[i+j+1] > cos(threshold * 3.1415962 / 180))
					judge=0;
			}
			if (judge)
				choosen = 1;
		}
		double djudge = cos(threshold * 3.1415962/180);
		if (i > nrnextp)
		{
			int judge = 1;
			for (int j = 0; j < nrnextp; j++)
			{
				if (cos_angle_array[i -1-j] > cos(threshold * 3.1415962 / 180))
					judge =0;
			}
			if (judge)
				choosen = 1;
		}
		if (choosen)
		{
			cloud_in.points[count].x = cloud_in.points[i].x;
			cloud_in.points[count].y = cloud_in.points[i].y;
			cloud_in.points[count].z = cloud_in.points[i].z;
			count++;
		}
	}
	cloud_in.width = count;
	cloud_in.points.resize(count);
	return 0;

}