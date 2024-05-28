#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/extract_indices.h>
#include <primitives_map.h>
#include "pcl_search.h"
using namespace std;
using namespace pcl;
using namespace pcl::octree;

/// <summary>
/// K��������
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="referencePoint">�ο������</param>
/// <param name="k">��������</param>
/// <returns>����㼯</returns>
Point3Fs* kSearch(Point3F points[], const int length, const Point3F referencePoint, const int k)
{
	const PointCloud<PointXYZ>::Ptr sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();
	const PointXYZ& point = pclsharp::toPointXYZ(referencePoint);

	const IndicesPtr pointIndices = std::make_shared<Indices>();	//�������Ľ��ڵ������
	vector<float> pointSqrDistances;								//�ο����Ӧ���ڵ�ľ����ƽ��
	search::KdTree<PointXYZ> kdTree;
	kdTree.setInputCloud(sourceCloud);
	const int& neighborsCount = kdTree.nearestKSearch(point, k, *pointIndices, pointSqrDistances);
	if (neighborsCount > 0)
	{
		ExtractIndices<PointXYZ> extractor;
		extractor.setInputCloud(sourceCloud);
		extractor.setIndices(pointIndices);
		extractor.setNegative(false);
		extractor.filter(*targetCloud);
	}

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// �뾶����
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="referencePoint">�ο������</param>
/// <param name="radius">�����뾶</param>
/// <returns>����㼯</returns>
Point3Fs* CALLING_MODE radiusSearch(Point3F points[], const int length, const Point3F referencePoint, const float radius)
{
	const PointCloud<PointXYZ>::Ptr sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();
	const PointXYZ& point = pclsharp::toPointXYZ(referencePoint);

	const IndicesPtr pointIndices = std::make_shared<Indices>();	//�������Ľ��ڵ������
	vector<float> pointSqrDistances;								//�ο����Ӧ���ڵ�ľ����ƽ��
	search::KdTree<PointXYZ> kdTree;
	kdTree.setInputCloud(sourceCloud);
	const int& neighborsCount = kdTree.radiusSearch(point, radius, *pointIndices, pointSqrDistances);
	if (neighborsCount > 0)
	{
		ExtractIndices<PointXYZ> extractor;
		extractor.setInputCloud(sourceCloud);
		extractor.setIndices(pointIndices);
		extractor.setNegative(false);
		extractor.filter(*targetCloud);
	}

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// �˲�������
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="referencePoint">�ο������</param>
/// <param name="resolution">�ֱ���</param>
/// <returns>����㼯</returns>
Point3Fs* octreeSearch(Point3F points[], const int length, const Point3F referencePoint, const float resolution)
{
	const PointCloud<PointXYZ>::Ptr sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();
	const PointXYZ& point = pclsharp::toPointXYZ(referencePoint);

	const IndicesPtr pointIndices = std::make_shared<Indices>();	//�������Ľ��ڵ������
	OctreePointCloudSearch<PointXYZ> octree = OctreePointCloudSearch<PointXYZ>(resolution);
	octree.setInputCloud(sourceCloud);
	octree.addPointsFromInputCloud();
	const int& neighborsCount = octree.voxelSearch(point, *pointIndices);
	if (neighborsCount > 0)
	{
		ExtractIndices<PointXYZ> extractor;
		extractor.setInputCloud(sourceCloud);
		extractor.setIndices(pointIndices);
		extractor.setNegative(false);
		extractor.filter(*targetCloud);
	}

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}
