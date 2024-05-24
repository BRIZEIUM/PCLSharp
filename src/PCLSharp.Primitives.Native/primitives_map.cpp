#include "primitives_map.h"
using namespace std;
using namespace pcl;

/// <summary>
/// ����㼯ӳ�����
/// </summary>
/// <param name="point3Fs">����㼯</param>
/// <param name="length">����</param>
/// <returns>����</returns>
PointCloud<PointXYZ>::Ptr pclsharp::toPointCloud(Point3F point3Fs[], const int& length)
{
	PointCloud<PointXYZ>::Ptr pointXYZs = std::make_shared<PointCloud<PointXYZ>>();
	for (int i = 0; i < length; i++)
	{
		const Point3F& point3F = point3Fs[i];
		const PointXYZ& pointXYZ = PointXYZ(point3F.X, point3F.Y, point3F.Z);
		pointXYZs->push_back(pointXYZ);
	}

	return pointXYZs;
}

/// <summary>
/// ����ӳ������㼯�ṹ��
/// </summary>
/// <param name="pointCloud">����</param>
/// <returns>����㼯�ṹ��</returns>
Point3Fs* pclsharp::toPoint3Fs(const PointCloud<PointXYZ>& pointCloud)
{
	const size_t length = pointCloud.size();
	Point3F* points = new Point3F[length];
	for (int i = 0; i < length; i++)
	{
		const PointXYZ& pointXYZ = pointCloud.points[i];
		points[i] = Point3F(pointXYZ.x, pointXYZ.y, pointXYZ.z);
	}

	Point3Fs* point3Fs = new Point3Fs(points, static_cast<int>(length));

	return point3Fs;
}

/// <summary>
/// ��������ӳ�����
/// </summary>
/// <param name="normal3Fs">��������</param>
/// <param name="length">����</param>
/// <returns>����</returns>
PointCloud<Normal>::Ptr pclsharp::toPointCloud(Normal3F normal3Fs[], const int& length)
{
	PointCloud<Normal>::Ptr normals = std::make_shared<PointCloud<Normal>>();
	for (int i = 0; i < length; i++)
	{
		const Normal3F& normal3F = normal3Fs[i];
		const Normal& normal = Normal(normal3F.NX, normal3F.NY, normal3F.NZ);
		normals->push_back(normal);
	}

	return normals;
}

/// <summary>
/// ����ӳ�䷨�������ṹ��
/// </summary>
/// <param name="pointCloud">����</param>
/// <returns>���������ṹ��</returns>
Normal3Fs* pclsharp::toNormal3Fs(const PointCloud<Normal>& pointCloud)
{
	const size_t length = pointCloud.size();
	Normal3F* normals = new Normal3F[length];
	for (int i = 0; i < length; i++)
	{
		const Normal& normal = pointCloud.points[i];
		normals[i] = Normal3F(normal.normal_x, normal.normal_y, normal.normal_z);
	}

	Normal3Fs* normal3Fs = new Normal3Fs(normals, static_cast<int>(length));

	return normal3Fs;
}

/// <summary>
/// ����㷨������ӳ�����
/// </summary>
/// <param name="point3Normal3s">����㷨������</param>
/// <param name="length">����</param>
/// <returns>����</returns>
PointCloud<PointNormal>::Ptr pclsharp::toPointCloud(Point3Normal3 point3Normal3s[], const int& length)
{
	PointCloud<PointNormal>::Ptr pointNormals = std::make_shared<PointCloud<PointNormal>>();
	for (int i = 0; i < length; i++)
	{
		const Point3Normal3& point3Normal3 = point3Normal3s[i];
		const PointNormal& pointNormal = PointNormal(point3Normal3.X, point3Normal3.Y, point3Normal3.Z, point3Normal3.NX, point3Normal3.NY, point3Normal3.NZ);
		pointNormals->push_back(pointNormal);
	}

	return pointNormals;
}

/// <summary>
/// ����ӳ������㷨�������ṹ��
/// </summary>
/// <param name="pointCloud">����</param>
/// <returns>����㷨�������ṹ��</returns>
Point3Normal3s* pclsharp::toPoint3Normal3s(const PointCloud<PointNormal>& pointCloud)
{
	const size_t length = pointCloud.size();
	Point3Normal3* pointNormals = new Point3Normal3[length];
	for (int i = 0; i < length; i++)
	{
		const PointNormal& pointNormal = pointCloud.points[i];
		pointNormals[i] = Point3Normal3(pointNormal.x, pointNormal.y, pointNormal.z, pointNormal.normal_x, pointNormal.normal_y, pointNormal.normal_z);
	}

	Point3Normal3s* point3Normal3s = new Point3Normal3s(pointNormals, static_cast<int>(length));

	return point3Normal3s;
}

/// <summary>
/// �������ɫ��ӳ�����
/// </summary>
/// <param name="point3Color4s">�������ɫ��</param>
/// <param name="length">����</param>
/// <returns>����</returns>
PointCloud<PointXYZRGBA>::Ptr pclsharp::toPointCloud(Point3Color4 point3Color4s[], const int& length)
{
	PointCloud<PointXYZRGBA>::Ptr pointColors = std::make_shared<PointCloud<PointXYZRGBA>>();
	for (int i = 0; i < length; i++)
	{
		const Point3Color4& point3Color4 = point3Color4s[i];
		const PointXYZRGBA& pointColor = PointXYZRGBA(point3Color4.X, point3Color4.Y, point3Color4.Z, point3Color4.R, point3Color4.G, point3Color4.B, point3Color4.A);
		pointColors->push_back(pointColor);
	}

	return pointColors;
}

/// <summary>
/// ����ӳ���������ɫ���ṹ��
/// </summary>
/// <param name="pointCloud">����</param>
/// <returns>�������ɫ���ṹ��</returns>
Point3Color4s* pclsharp::toPoint3Color4s(const PointCloud<PointXYZRGBA>& pointCloud)
{
	const size_t length = pointCloud.size();
	Point3Color4* pointColors = new Point3Color4[length];
	for (int i = 0; i < length; i++)
	{
		const PointXYZRGBA& pointColor = pointCloud.points[i];
		pointColors[i] = Point3Color4(pointColor.x, pointColor.y, pointColor.z, pointColor.r, pointColor.g, pointColor.b, pointColor.a);
	}

	Point3Color4s* point3Color4s = new Point3Color4s(pointColors, static_cast<int>(length));

	return point3Color4s;
}
