#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
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
	const PointCloud<PointXYZ>::Ptr sourceCloud = std::make_shared<PointCloud<PointXYZ>>();
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//���ص���
	const PointCloud<PointXYZ>& pointCloud = pclsharp::toPointCloud(points, length);
	copyPointCloud(pointCloud, *sourceCloud);

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
/// ���þ��Ȳ���
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="radius">�����뾶</param>
/// <returns>���˺�㼯</returns>
Point3Fs* applyUniformSampling(Point3F points[], const int length, const float radius)
{
	const PointCloud<PointXYZ>::Ptr sourceCloud = std::make_shared<PointCloud<PointXYZ>>();
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//���ص���
	const PointCloud<PointXYZ>& pointCloud = pclsharp::toPointCloud(points, length);
	copyPointCloud(pointCloud, *sourceCloud);

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
/// <param name="leafSize">Ҷ�ߴ�</param>
/// <returns>���˺�㼯</returns>
Point3Fs* applyVoxelGrid(Point3F points[], const int length, const float leafSize)
{
	const PointCloud<PointXYZ>::Ptr sourceCloud = std::make_shared<PointCloud<PointXYZ>>();
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//���ص���
	const PointCloud<PointXYZ>& pointCloud = pclsharp::toPointCloud(points, length);
	copyPointCloud(pointCloud, *sourceCloud);

	//���ؽ�����
	VoxelGrid<PointXYZ> voxelGrid;
	voxelGrid.setInputCloud(sourceCloud);
	voxelGrid.setLeafSize(leafSize, leafSize, leafSize);
	voxelGrid.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// ������Ⱥ���Ƴ�
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="meanK">ƽ��������Ƶ�����ھӵ�����</param>
/// <param name="stddevMult">��׼����ֵϵ��</param>
/// <returns>���˺�㼯</returns>
Point3Fs* applyOutlierRemoval(Point3F points[], const int length, const int meanK, const float stddevMult)
{
	const PointCloud<PointXYZ>::Ptr sourceCloud = std::make_shared<PointCloud<PointXYZ>>();
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//���ص���
	const PointCloud<PointXYZ>& pointCloud = pclsharp::toPointCloud(points, length);
	copyPointCloud(pointCloud, *sourceCloud);

	//��ʼ��ͳ��ѧ��Ⱥ���Ƴ�������
	StatisticalOutlierRemoval<PointXYZ> statisticalOutlierRemoval;
	statisticalOutlierRemoval.setInputCloud(sourceCloud);
	statisticalOutlierRemoval.setMeanK(50);//����ƽ��������Ƶ�����ھӵ�����K	
	statisticalOutlierRemoval.setStddevMulThresh(1.0);//���ñ�׼����ֵϵ��
	statisticalOutlierRemoval.filter(*targetCloud);	//ִ�й���

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// �ͷ���Դ
/// </summary>
/// <param name="pointer">ָ��</param>
void dispose(const Point3Fs* pointer)
{
	delete pointer;
}
