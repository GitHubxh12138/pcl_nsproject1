#include "txtfile.h"


static pcl::PointXYZ cloud_in_shift = { 0,0,0 };;//点云整体平移参数
static int cloud_in_shift_exist = 0;

//获得txt文件的行数
int getNumberOfEdges(const char* file_name)
{
	char flag;
	int edgeNum, count = 0;
	FILE* fp = fopen(file_name, "rt+");//open the TXT file, can only read, cannot write
	while (!feof(fp)) {

		flag = fgetc(fp);
		if (flag == '\n') count++;
	}

	edgeNum = count;//因为最后一行没有换行符\n，所以需要在count补加1,由于数据多一行，不加
	fclose(fp);
	//std::cout << " the edge number is " << edgeNum << std::endl;
	return edgeNum;

}

int getCmdRes(const char* sc)

{
	char command[100];
	sprintf(command, "dir \/b %s | find \/v \/c \"::\"", sc);
	FILE* crs;
	crs = _popen(command, "r"); // execute the command 

	char result[1024] = "0";

	fread(result, sizeof(char), sizeof(result), crs);


	if (NULL != crs)

	{

		fclose(crs);

		crs = NULL;

	}


	std::string res = result;
	std::stringstream stream;     //声明一个stringstream变量
	int n;
	stream << res;
	stream >> n;

	return n;

}



//读txt文件,保存在cloud中，第一次读将取第一个点的整数部分做整体平移参数
void ReadTxtFile(const char* file_name, pcl::PointCloud<pcl::PointXYZ>& cloud)
{

	if (cloud_in_shift_exist == 0)
	{
		cloud.width = getNumberOfEdges(file_name);
		cloud.height = 1;
		cloud.is_dense = false;
		cloud.points.resize(cloud.width * cloud.height);

		std::ifstream file_in;
		file_in.open(file_name);
		char h;
		double d_x, d_y, d_z;
		int i_x, i_y, i_z;
		
		file_in >> d_x >> h;
		file_in >> d_y >> h;
		file_in >> d_z;
		i_x = d_x;
		i_y = d_y;
		i_z = d_z;
		cloud_in_shift.x = i_x;
		cloud_in_shift.y = i_y;
		cloud_in_shift.z = i_z;
		file_in.seekg(0);

		for (size_t i = 0; i < cloud.points.size(); i++) 
		{
			file_in >> d_x >> h;
			file_in >> d_y >> h;
			file_in >> d_z;
			cloud[i].x = d_x - cloud_in_shift.x;
			cloud[i].y = d_y - cloud_in_shift.y;
			cloud[i].z = d_z - cloud_in_shift.z;

		}
		cloud_in_shift_exist = 1;
	}
	else
	{
		cloud.width = getNumberOfEdges(file_name);
		cloud.height = 1;
		cloud.is_dense = false;
		cloud.points.resize(cloud.width * cloud.height);

		std::ifstream file_in;
		file_in.open(file_name);
		char h;
		double d_x, d_y, d_z;




		for (size_t i = 0; i < cloud.points.size(); i++)
		{
			file_in >> d_x >> h;
			file_in >> d_y >> h;
			file_in >> d_z;
			cloud[i].x = d_x - cloud_in_shift.x;
			cloud[i].y = d_y - cloud_in_shift.y;
			cloud[i].z = d_z - cloud_in_shift.z;

		}
	}
	



}




//写txt文件
int WriteTxtFile(const char* file_name, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
	if (cloud.width)
	{
	std::ofstream file_out;
	file_out.open(file_name);
	//file_out.precision(10);
	for (int i = 0; i < cloud.width; i++)
	{
		file_out <<std::fixed<<std::setprecision(3)<< ((double)cloud.points[i].x+ (double)cloud_in_shift.x) << "," << (double)cloud.points[i].y + (double)cloud_in_shift.y << "," << (double)cloud.points[i].z + (double)cloud_in_shift.z << std::endl;
	}
	}
	
	return 0;
}

int OnlyWriteTxtFile(const char* file_name, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
	if (cloud.width)
	{
		std::ofstream file_out;
		file_out.open(file_name,std::ios_base::app);
		//file_out.precision(10);
		for (int i = 0; i < cloud.width; i++)
		{
			file_out << std::fixed << std::setprecision(3) << ((double)cloud.points[i].x + (double)cloud_in_shift.x) << "," << (double)cloud.points[i].y + (double)cloud_in_shift.y << "," << (double)cloud.points[i].z + (double)cloud_in_shift.z << std::endl;
		}
	}

	return 0;
}