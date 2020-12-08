#include"fileprocess.h"

#define Detect_Number 4//���̽��ĵ�ĸ���
#define Threshold 0.6//�߳��ݶȲ�
#define Connected_Distance 0.18//��ͨ��������
#define Connected_Number 25//��ͨ�������ٵ���
#define Fitting_Detect_Number 10
#define Fitting_Max_Distance 1
#define Filter_Max_distance 0.01
#define Flodernum 1


void MoveCloud(pcl::PointCloud<pcl::PointXYZ>& cloud_in, pcl::PointXYZ P, int num)
{
	for (int i = 0; i < cloud_in.width; i++)
	{
		cloud_in.points[i].x = cloud_in.points[i].x - num * P.x;
		cloud_in.points[i].y = cloud_in.points[i].y - num * P.y;
		cloud_in.points[i].z = cloud_in.points[i].z - num * P.z;
	}
}


int seedselect(char* path, char* fname)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in2(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointXYZ P1 = { 574337.343140 ,3618866.747589 ,135.665997 };
	pcl::PointXYZ P2 = { 574321.190399 ,3618850.358170 ,135.515419 };
	pcl::PointXYZ PM;
	double Disp = std::sqrt(pow((P1.x - P2.x), 2) + pow((P1.y - P2.y), 2) + pow((P1.z - P2.z), 2));
	PM.x = (P2.x - P1.x) * 0.5 / Disp;
	PM.y = (P2.y - P1.y) * 0.5 / Disp;
	PM.z = (P2.z - P1.z) * 0.5 / Disp;

	fstream file1("cloud.txt", ios::out);
	fstream file2("seed.txt", ios::out);

	char FileName[100];
	char folderpath[100];
	sprintf(folderpath, "%sfirst\\%d", path, Flodernum);
	int nrfile = getCmdRes(folderpath);
	for (int j = 0; j < 20; j++)
	{
		//��txt�ļ�
		sprintf(FileName, "%sfirst\\%d\\%s%d%s", path, Flodernum, fname, j + 1, ".txt");
		ReadTxtFile(FileName, *cloud_in1);


		NextDirectionFilter(*cloud_in1, Detect_Number, Threshold);//���Ƹ߳��ݶ��˲�
		LabelConnectedComponent(*cloud_in1, Connected_Distance, 15);//������ͨ����
		MoveCloud(*cloud_in1, PM, j);
		OnlyWriteTxtFile("cloud.txt", *cloud_in1);
	}
	ReadTxtFile("cloud.txt", *cloud_in1);
	sprintf(FileName, "H:\\first\\%d\\cross_1.txt", Flodernum);
	ReadTxtFile(FileName, *cloud_in2);

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_in1);
	std::vector<int>pointIdxRadiusSearch;
	std::vector<float>pointRadiusSquaredDistance;
	int Pnum(0);
	int count = 0;
	while (Pnum < cloud_in2->width)
	{
		int test = kdtree.radiusSearch(cloud_in2->points[Pnum], 0.1, pointIdxRadiusSearch, pointRadiusSquaredDistance);
		if (kdtree.radiusSearch(cloud_in2->points[Pnum], 0.1, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 95)
		{
			cloud_in2->points[count].x = cloud_in2->points[Pnum].x;
			cloud_in2->points[count].y = cloud_in2->points[Pnum].y;
			cloud_in2->points[count].z = cloud_in2->points[Pnum].z;
			count++;
		}
		Pnum += 10;
	}
	cloud_in2->width = count;
	cloud_in2->points.resize(count);



	OnlyWriteTxtFile("seed.txt", *cloud_in2);









	return 0;
}

void SingleFileProcess()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out2(new pcl::PointCloud<pcl::PointXYZ>);
	

	ReadTxtFile("H:\\first\\1\\cross_39.txt",*cloud_in1);//��txt�ļ�
	ReadTxtFile("H:\\second\\1\\cross_39.txt", *cloud_in2);
	*cloud_out1 = *cloud_in1;//���Ƶ���
	*cloud_out2 = *cloud_in2;

	
	



	NextDirectionFilter(*cloud_in1, Detect_Number, Threshold);//���Ƹ߳��ݶ��˲�
	LabelConnectedComponent(*cloud_in1, Connected_Distance, Connected_Number);//������ͨ����

	Connnected_Select(cloud_in1, cloud_out1, 0.05);
	Connnected_Select(cloud_in1, cloud_out2, 0.05);

	WriteTxtFile("һ��39��ȡ���.txt", *cloud_out1);//���
	WriteTxtFile("����39��ȡ���.txt", *cloud_out2);//���



	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaxCorrespondenceDistance(0.05);
	icp.setMaximumIterations(100);
	icp.setInputSource(cloud_out2);
	icp.setInputTarget(cloud_out1);
	icp.align(*cloud_out2);


	
	WriteTxtFile("����39��׼���.txt", *cloud_out2);//���

	Monitoring_Fittingline(cloud_out1, cloud_out2, 0.01, 10);

	WriteTxtFile("����39���������.txt", *cloud_out2);//���
	LabelConnectedComponent(*cloud_out2, 0.1, 5);//������ͨ����
	WriteTxtFile("����39������ս��.txt", *cloud_out2);//���
	
}


void BatchProcessingSeed()
{
	char PathOfTxtFile[30] = "H:\\";
	char FrontName[20] = "cross_";
	seedselect(PathOfTxtFile, FrontName);


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seed(new pcl::PointCloud<pcl::PointXYZ>);
	ReadTxtFile("seed.txt", *cloud_seed);

	pcl::PointXYZ P1 = { 574337.343140 ,3618866.747589 ,135.665997 };
	pcl::PointXYZ P2 = { 574321.190399 ,3618850.358170 ,135.515419 };
	pcl::PointXYZ PM;
	double Disp = std::sqrt(pow((P1.x - P2.x), 2) + pow((P1.y - P2.y), 2) + pow((P1.z - P2.z), 2));
	PM.x = (P2.x - P1.x) * 0.5 / Disp;
	PM.y = (P2.y - P1.y) * 0.5 / Disp;
	PM.z = (P2.z - P1.z) * 0.5 / Disp;

	fstream file1("FILTERRESULT.txt", ios::out);
	fstream file2("RESULT.txt", ios::out);

	char FileName[100];
	char folderpath[100];
	sprintf(folderpath, "%sfirst\\%d", PathOfTxtFile, 1);
	int nrfile = getCmdRes(folderpath);
	for (int j = 0; j < nrfile; j++)
	{
		//��txt�ļ�
		sprintf(FileName, "%sfirst\\%d\\%s%d%s", PathOfTxtFile, 1, FrontName, j + 1, ".txt");
		ReadTxtFile(FileName, *cloud_in1);
		sprintf(FileName, "%ssecond\\%d\\%s%d%s", PathOfTxtFile, 1, FrontName, j + 1, ".txt");
		ReadTxtFile(FileName, *cloud_in2);

		*cloud_out1 = *cloud_in1;//���Ƶ���
		*cloud_out2 = *cloud_in2;


		NextDirectionFilter(*cloud_out1, Detect_Number, Threshold);//���Ƹ߳��ݶ��˲�
		NextDirectionFilter(*cloud_out2, Detect_Number, Threshold);//���Ƹ߳��ݶ��˲�
		 //AngleFilter(*cloud_out1, 10, 145);
		 //AngleFilter(*cloud_out2, 10, 145);
		LabelConnectedComponent(*cloud_out1, Connected_Distance, 25);//������ͨ����



		OnlyWriteTxtFile("FILTERRESULT.txt", *cloud_out1);
		Connnected_Select(cloud_out1, cloud_out2, 0.05);



	//icp
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setMaximumIterations(100);
		icp.setMaxCorrespondenceDistance(0.05);
		icp.setInputSource(cloud_out2);
		icp.setInputTarget(cloud_out1);
		icp.align(*cloud_out2);



		Monitoring_Fittingline(cloud_out1, cloud_out2, 0.01, 4);
		LabelConnectedComponent(*cloud_out2, 0.1, 10);//������ͨ����



		if (cloud_out2->width)
		{
			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
			kdtree.setInputCloud(cloud_seed);
			int k = 1;
			std::vector<int>pointIdxNKNSearch;
			std::vector<float>pointNKNSquaredDistance;


			int count = 0;
			for (int i = 0; i < cloud_out2->width; i++)
			{
				kdtree.nearestKSearch(cloud_out2->points[i], k, pointIdxNKNSearch, pointNKNSquaredDistance);
				if (pointNKNSquaredDistance[0] < std::pow(0.4, 2))
				{
					cloud_out2->points[count] = cloud_out2->points[i];
					count++;
				}
			}
			cloud_out2->width = count;
			cloud_out2->points.resize(count);
		}


		MoveCloud(*cloud_seed, PM, -1);


		OnlyWriteTxtFile("RESULT.txt", *cloud_out2);
	}


}

void BatchProcessingNeighbor()
{
	char PathOfTxtFile[30] = "H:\\";
	char FrontName[20] = "cross_";
	fstream file2("RESULT.txt", ios::out);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in2(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out1(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out2(new pcl::PointCloud<pcl::PointXYZ>);

	char FileName[100];
	char folderpath[100];
	sprintf(folderpath, "%sfirst\\%d", PathOfTxtFile, 1);
	int nrfile = getCmdRes(folderpath);
	for (int j = 0; j < nrfile; j++)
	{
		//��txt�ļ�
		sprintf(FileName, "%sfirst\\%d\\%s%d%s", PathOfTxtFile, 1, FrontName, j + 1, ".txt");
		ReadTxtFile(FileName, *cloud_in1);
		sprintf(FileName, "%ssecond\\%d\\%s%d%s", PathOfTxtFile, 1, FrontName, j + 1, ".txt");
		ReadTxtFile(FileName, *cloud_in2);


		NextDirectionFilter(*cloud_in1, Detect_Number, Threshold);//���Ƹ߳��ݶ��˲�
		NextDirectionFilter(*cloud_in2, Detect_Number, Threshold);//���Ƹ߳��ݶ��˲�
		 //AngleFilter(*cloud_out1, 10, 145);
		 //AngleFilter(*cloud_out2, 10, 145);
		LabelConnectedComponent(*cloud_in1, Connected_Distance, 25);//������ͨ����
		sprintf(FileName, "./first/1/filterresult%d.txt", j + 1);
		WriteTxtFile(FileName, *cloud_in1);
		Connnected_Select(cloud_in1, cloud_in2, 0.05);
		sprintf(FileName, "./second/1/filterresult%d.txt", j + 1);
		WriteTxtFile(FileName, *cloud_in2);


	}
	for (int j = 0; j < nrfile; j++)
	{
		sprintf(FileName, "./first/1/filterresult%d.txt", j + 1);
		ReadTxtFile(FileName, *cloud_in1);
		sprintf(FileName, "./second/1/filterresult%d.txt", j + 1);
		ReadTxtFile(FileName, *cloud_in2);

		//icp
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setMaximumIterations(100);
		icp.setMaxCorrespondenceDistance(0.05);
		icp.setInputSource(cloud_in2);
		icp.setInputTarget(cloud_in1);
		icp.align(*cloud_in2);



		Monitoring_Fittingline(cloud_in1, cloud_in2, 0.01, 4);
		//NeighborLineAnalysisoneline(*cloud_in2, 0.1, 10, j+1, nrfile);
		NeighborLineAnalysistwoline(*cloud_in2, 0.1, 10, j + 1, nrfile);

		if (cloud_in2->width)
		{
			OnlyWriteTxtFile("RESULT.txt", *cloud_in2);
		}


	}

}
