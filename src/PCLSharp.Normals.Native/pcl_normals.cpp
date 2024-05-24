#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <primitives_map.h>
#include "pcl_normals.h"
using namespace pcl;

/// <summary>
/// ���㷨����
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="k">������������</param>
/// <returns>��������</returns>
Normal3Fs* estimateNormalsByK(Point3F points[], const int length, const int k)
{
	const PointCloud<PointXYZ>::Ptr& cloud = pclsharp::toPointCloud(points, length);
	const PointCloud<Normal>::Ptr cloudNormals = std::make_shared<PointCloud<Normal>>();

	//���㷨����
	const search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();
	NormalEstimation<PointXYZ, Normal> normalEstimator;
	normalEstimator.setInputCloud(cloud);
	normalEstimator.setSearchMethod(kdTree);
	normalEstimator.setKSearch(k);
	normalEstimator.compute(*cloudNormals);

	Normal3Fs* normal3Fs = pclsharp::toNormal3Fs(*cloudNormals);

	return normal3Fs;
}

/// <summary>
/// ���㷨����
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="radius">�����뾶</param>
/// <returns>��������</returns>
Normal3Fs* estimateNormalsByRadius(Point3F points[], const int length, const float radius)
{
	const PointCloud<PointXYZ>::Ptr& cloud = pclsharp::toPointCloud(points, length);
	const PointCloud<Normal>::Ptr cloudNormals = std::make_shared<PointCloud<Normal>>();

	//���㷨����
	const search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();
	NormalEstimation<PointXYZ, Normal> normalEstimator;
	normalEstimator.setInputCloud(cloud);
	normalEstimator.setSearchMethod(kdTree);
	normalEstimator.setRadiusSearch(radius);
	normalEstimator.compute(*cloudNormals);

	Normal3Fs* normal3Fs = pclsharp::toNormal3Fs(*cloudNormals);

	return normal3Fs;
}

/// <summary>
/// ���㷨���� (OMP)
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="k">������������</param>
/// <returns>��������</returns>
Normal3Fs* estimateNormalsByKP(Point3F points[], const int length, const int k)
{
	const PointCloud<PointXYZ>::Ptr& cloud = pclsharp::toPointCloud(points, length);
	const PointCloud<Normal>::Ptr cloudNormals = std::make_shared<PointCloud<Normal>>();

	//���㷨����
	const search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();
	NormalEstimationOMP<PointXYZ, Normal> normalEstimator;
	normalEstimator.setInputCloud(cloud);
	normalEstimator.setSearchMethod(kdTree);
	normalEstimator.setKSearch(k);
	normalEstimator.compute(*cloudNormals);

	Normal3Fs* normal3Fs = pclsharp::toNormal3Fs(*cloudNormals);

	return normal3Fs;
}

/// <summary>
/// ���㷨���� (OMP)
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="radius">�����뾶</param>
/// <returns>��������</returns>
Normal3Fs* estimateNormalsByRadiusP(Point3F points[], const int length, const float radius)
{
	const PointCloud<PointXYZ>::Ptr& cloud = pclsharp::toPointCloud(points, length);
	const PointCloud<Normal>::Ptr cloudNormals = std::make_shared<PointCloud<Normal>>();

	//���㷨����
	const search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();
	NormalEstimationOMP<PointXYZ, Normal> normalEstimator;
	normalEstimator.setInputCloud(cloud);
	normalEstimator.setSearchMethod(kdTree);
	normalEstimator.setRadiusSearch(radius);
	normalEstimator.compute(*cloudNormals);

	Normal3Fs* normal3Fs = pclsharp::toNormal3Fs(*cloudNormals);

	return normal3Fs;
}

/// <summary>
/// ��������
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <returns>���������</returns>
Point3F* estimateCentroid(Point3F points[], const int length)
{
	//���ص���
	const PointCloud<PointXYZ>::Ptr& cloud = pclsharp::toPointCloud(points, length);

	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloud, centroid);

	Point3F* point3F = new Point3F(centroid[0], centroid[1], centroid[2]);

	return point3F;
}
