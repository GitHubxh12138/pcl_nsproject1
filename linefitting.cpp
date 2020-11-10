#include"linefitting.h"

double LineFittingReDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointXYZ point, float Max_Distance)
{
    double point_to_ling_distance;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers表示误差能容忍的点 记录的是点云的序号
    pcl::SACSegmentation<pcl::PointXYZ> seg;     // 创建一个分割器
    seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状
    seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
    seg.setDistanceThreshold(Max_Distance);         //设置误差容忍范围，也就是阈值
    seg.setInputCloud(cloud_in);               //输入点云
    seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量
    //打印平面方程
    /*std::cout << "a：" << coefficients->values[0] << endl;
    std::cout << "b：" << coefficients->values[1] << endl;
    std::cout << "c：" << coefficients->values[2] << endl;
    std::cout << "d：" << coefficients->values[3] << endl;
    std::cout << "e：" << coefficients->values[4] << endl;
    std::cout << "f：" << coefficients->values[5] << endl;*/
    point_to_ling_distance = std::sqrt(std::pow(point.x - coefficients->values[0], 2) + std::pow(point.y - coefficients->values[1], 2) + std::pow(point.z - coefficients->values[2], 2) -
        std::pow((point.x - coefficients->values[0]) * coefficients->values[3] + (point.y - coefficients->values[1]) * coefficients->values[4] + (point.z - coefficients->values[2]) * coefficients->values[5], 2));
    return point_to_ling_distance;
}