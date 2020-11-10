#include"linefitting.h"

double LineFittingReDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointXYZ point, float Max_Distance)
{
    double point_to_ling_distance;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers��ʾ��������̵ĵ� ��¼���ǵ��Ƶ����
    pcl::SACSegmentation<pcl::PointXYZ> seg;     // ����һ���ָ���
    seg.setOptimizeCoefficients(true);      // Optional��������ÿ���ѡ�����ƽ��չʾ�ĵ��Ƿָ���ĵ㻹�Ƿָ�ʣ�µĵ㡣
    seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-����Ŀ�꼸����״
    seg.setMethodType(pcl::SAC_RANSAC);     //�ָ�������������
    seg.setDistanceThreshold(Max_Distance);         //����������̷�Χ��Ҳ������ֵ
    seg.setInputCloud(cloud_in);               //�������
    seg.segment(*inliers, *coefficients);   //�ָ���ƣ����ƽ��ͷ�����
    //��ӡƽ�淽��
    /*std::cout << "a��" << coefficients->values[0] << endl;
    std::cout << "b��" << coefficients->values[1] << endl;
    std::cout << "c��" << coefficients->values[2] << endl;
    std::cout << "d��" << coefficients->values[3] << endl;
    std::cout << "e��" << coefficients->values[4] << endl;
    std::cout << "f��" << coefficients->values[5] << endl;*/
    point_to_ling_distance = std::sqrt(std::pow(point.x - coefficients->values[0], 2) + std::pow(point.y - coefficients->values[1], 2) + std::pow(point.z - coefficients->values[2], 2) -
        std::pow((point.x - coefficients->values[0]) * coefficients->values[3] + (point.y - coefficients->values[1]) * coefficients->values[4] + (point.z - coefficients->values[2]) * coefficients->values[5], 2));
    return point_to_ling_distance;
}