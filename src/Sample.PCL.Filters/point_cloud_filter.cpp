#include "point_cloud_filter.h"
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
PointArray* applyPassThrogh(Point3F points[], const int length, const char* axis, const float limitMin, const float limixMax)
{
	const PointCloud<PointXYZ>::Ptr sourceCloud = std::make_shared<PointCloud<PointXYZ>>();
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//���ص���
	for (int i = 0; i < length; i++)
	{
		const Point3F& point = points[i];
		PointXYZ pointXYZ = PointXYZ(point.X, point.Y, point.Z);
		sourceCloud->points.push_back(pointXYZ);
	}

	PassThrough<PointXYZ> passThrough;
	passThrough.setInputCloud(sourceCloud);
	passThrough.setFilterFieldName(axis);				//���ù���������
	passThrough.setFilterLimits(limitMin, limixMax);	//���ù��˷�Χ
	passThrough.filter(*targetCloud);

	const int outLength = targetCloud->size();
	PointArray* pointArray = new PointArray();
	pointArray->Length = outLength;
	pointArray->Points = new Point3F[outLength];
	for (int i = 0; i < outLength; i++)
	{
		const PointXYZ& pointXYZ = targetCloud->points[i];
		pointArray->Points[i] = Point3F(pointXYZ.x, pointXYZ.y, pointXYZ.z);
	}

	return pointArray;
}

/// <summary>
/// ���þ��Ȳ���
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="radius">�����뾶</param>
/// <returns>���˺�㼯</returns>
PointArray* applyUniformSampling(Point3F points[], const int length, const float radius)
{
	const PointCloud<PointXYZ>::Ptr sourceCloud = std::make_shared<PointCloud<PointXYZ>>();
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//���ص���
	for (int i = 0; i < length; i++)
	{
		const Point3F& point = points[i];
		PointXYZ pointXYZ = PointXYZ(point.X, point.Y, point.Z);
		sourceCloud->points.push_back(pointXYZ);
	}

	//��������������������
	UniformSampling<PointXYZ> uniformSampling;
	uniformSampling.setInputCloud(sourceCloud);
	uniformSampling.setRadiusSearch(radius);
	uniformSampling.filter(*targetCloud);

	const int outLength = targetCloud->size();
	PointArray* pointArray = new PointArray();
	pointArray->Length = outLength;
	pointArray->Points = new Point3F[outLength];
	for (int i = 0; i < outLength; i++)
	{
		const PointXYZ& pointXYZ = targetCloud->points[i];
		pointArray->Points[i] = Point3F(pointXYZ.x, pointXYZ.y, pointXYZ.z);
	}

	return pointArray;
}

/// <summary>
/// �������ؽ�����
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="leafSize">Ҷ�ߴ�</param>
/// <returns>���˺�㼯</returns>
PointArray* applyVoxelGrid(Point3F points[], const int length, const float leafSize)
{
	const PointCloud<PointXYZ>::Ptr sourceCloud = std::make_shared<PointCloud<PointXYZ>>();
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//���ص���
	for (int i = 0; i < length; i++)
	{
		const Point3F& point = points[i];
		PointXYZ pointXYZ = PointXYZ(point.X, point.Y, point.Z);
		sourceCloud->points.push_back(pointXYZ);
	}

	//����������������
	VoxelGrid<PointXYZ> voxelGrid;
	voxelGrid.setInputCloud(sourceCloud);
	voxelGrid.setLeafSize(leafSize, leafSize, leafSize);
	voxelGrid.filter(*targetCloud);

	const int outLength = targetCloud->size();
	PointArray* pointArray = new PointArray();
	pointArray->Length = outLength;
	pointArray->Points = new Point3F[outLength];
	for (int i = 0; i < outLength; i++)
	{
		const PointXYZ& pointXYZ = targetCloud->points[i];
		pointArray->Points[i] = Point3F(pointXYZ.x, pointXYZ.y, pointXYZ.z);
	}

	return pointArray;
}

/// <summary>
/// ������Ⱥ���Ƴ�
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="meanK">ƽ��������Ƶ�����ھӵ�����</param>
/// <param name="stddevMult">��׼����ֵϵ��</param>
/// <returns>���˺�㼯</returns>
PointArray* applyOutlierRemoval(Point3F points[], const int length, const int meanK, const float stddevMult)
{
	const PointCloud<PointXYZ>::Ptr sourceCloud = std::make_shared<PointCloud<PointXYZ>>();
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//���ص���
	for (int i = 0; i < length; i++)
	{
		const Point3F& point = points[i];
		PointXYZ pointXYZ = PointXYZ(point.X, point.Y, point.Z);
		sourceCloud->points.push_back(pointXYZ);
	}

	//��ʼ��ͳ��ѧ��Ⱥ���Ƴ�������
	StatisticalOutlierRemoval<PointXYZ> statisticalOutlierRemoval;
	statisticalOutlierRemoval.setInputCloud(sourceCloud);
	statisticalOutlierRemoval.setMeanK(50);//����ƽ��������Ƶ�����ھӵ�����K	
	statisticalOutlierRemoval.setStddevMulThresh(1.0);//���ñ�׼����ֵϵ��
	statisticalOutlierRemoval.filter(*targetCloud);	//ִ�й���

	const int outLength = targetCloud->size();
	PointArray* pointArray = new PointArray();
	pointArray->Length = outLength;
	pointArray->Points = new Point3F[outLength];
	for (int i = 0; i < outLength; i++)
	{
		const PointXYZ& pointXYZ = targetCloud->points[i];
		pointArray->Points[i] = Point3F(pointXYZ.x, pointXYZ.y, pointXYZ.z);
	}

	return pointArray;
}

/// <summary>
/// �ͷ���Դ
/// </summary>
/// <param name="pointer">ָ��</param>
void dispose(const PointArray* pointer)
{
	delete pointer;
}
