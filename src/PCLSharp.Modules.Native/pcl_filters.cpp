#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <primitives_map.h>
#include "pcl_filters.h"
using namespace std;
using namespace pcl;

/// <summary>
/// ����ֱͨ�˲�
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="axis">����������</param>
/// <param name="limitMin">���˷�Χ��Сֵ</param>
/// <param name="limixMax">���˷�Χ���ֵ</param>
/// <returns>���˺�㼯</returns>
Point3Fs* applyPassThrogh(Point3F points[], const int length, const char* axis, const float limitMin, const float limixMax)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//ֱͨ�˲�
	PassThrough<PointXYZ> passThrough;
	passThrough.setInputCloud(sourceCloud);
	passThrough.setFilterFieldName(axis);				//���ù���������
	passThrough.setFilterLimits(limitMin, limixMax);	//���ù��˷�Χ
	passThrough.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// �����������
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="seed">�������</param>
/// <param name="samplesCount">��������</param>
/// <returns>���˺�㼯</returns>
Point3Fs* applyRandomSampling(Point3F points[], const int length, const int seed, const int samplesCount)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//�������
	RandomSample<PointXYZ> randomSampling;
	randomSampling.setInputCloud(sourceCloud);
	randomSampling.setSeed(seed);
	randomSampling.setSample(samplesCount);
	randomSampling.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// ���þ��Ȳ���
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="radius">�����뾶</param>
/// <returns>���˺�㼯</returns>
Point3Fs* applyUniformSampling(Point3F points[], const int length, const float radius)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//���Ȳ���
	UniformSampling<PointXYZ> uniformSampling;
	uniformSampling.setInputCloud(sourceCloud);
	uniformSampling.setRadiusSearch(radius);
	uniformSampling.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// �������ؽ�����
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="leafSize">����ߴ�</param>
/// <returns>���˺�㼯</returns>
Point3Fs* applyVoxelGrid(Point3F points[], const int length, const float leafSize)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//���ؽ�����
	VoxelGrid<PointXYZ> voxelGrid;
	voxelGrid.setInputCloud(sourceCloud);
	voxelGrid.setLeafSize(leafSize, leafSize, leafSize);
	voxelGrid.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// ���ý������ؽ�����
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="leafSize">����ߴ�</param>
/// <returns>���˺�㼯</returns>
Point3Fs* applyApproxVoxelGrid(Point3F points[], int length, float leafSize)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//�������ؽ�����
	ApproximateVoxelGrid<PointXYZ> voxelGrid;
	voxelGrid.setInputCloud(sourceCloud);
	voxelGrid.setLeafSize(leafSize, leafSize, leafSize);
	voxelGrid.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// ����ͳ����Ⱥ���Ƴ�
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="meanK">ƽ��������Ƶ�����ھӵ�����</param>
/// <param name="stddevMult">��׼����ֵϵ��</param>
/// <returns>���˺�㼯</returns>
Point3Fs* applyStatisticalOutlierRemoval(Point3F points[], const int length, const int meanK, const float stddevMult)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//ͳ����Ⱥ���Ƴ�
	StatisticalOutlierRemoval<PointXYZ> statisticalOutlierRemoval;
	statisticalOutlierRemoval.setInputCloud(sourceCloud);
	statisticalOutlierRemoval.setMeanK(meanK);
	statisticalOutlierRemoval.setStddevMulThresh(stddevMult);
	statisticalOutlierRemoval.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// ���ð뾶��Ⱥ���Ƴ�
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="radius">�����뾶</param>
/// <param name="minNeighborsInRadius">�뾶��Χ�ڵ�������Сֵ</param>
/// <returns>���˺�㼯</returns>
Point3Fs* applyRadiusOutlierRemoval(Point3F points[], int length, const float radius, const int minNeighborsInRadius)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//�뾶��Ⱥ���Ƴ�
	RadiusOutlierRemoval<PointXYZ> radiusOutlierRemoval;
	radiusOutlierRemoval.setInputCloud(sourceCloud);
	radiusOutlierRemoval.setRadiusSearch(radius);
	radiusOutlierRemoval.setMinNeighborsInRadius(minNeighborsInRadius);
	radiusOutlierRemoval.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}
