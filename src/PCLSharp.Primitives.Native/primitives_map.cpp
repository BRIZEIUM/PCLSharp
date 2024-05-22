#include "primitives_map.h"
using namespace std;
using namespace pcl;

/// <summary>
/// �����ӳ��PointXYZ
/// </summary>
/// <param name="point3F">3D�����</param>
/// <returns>PointXYZ</returns>
PointXYZ pclsharp::toPointXYZ(const Point3F& point3F)
{
	const PointXYZ& pointXYZ = PointXYZ(point3F.X, point3F.Y, point3F.Z);

	return pointXYZ;
}

/// <summary>
/// ����㼯ӳ�����
/// </summary>
/// <param name="point3Fs">����㼯</param>
/// <param name="length">����</param>
/// <returns>����</returns>
PointCloud<PointXYZ> pclsharp::toPointCloud(Point3F point3Fs[], const int& length)
{
	PointCloud<PointXYZ> pointXYZs;
	for (int i = 0; i < length; i++)
	{
		const Point3F& point3F = point3Fs[i];
		const PointXYZ& pointXYZ = PointXYZ(point3F.X, point3F.Y, point3F.Z);
		pointXYZs.push_back(pointXYZ);
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
/// ������ӳ��Normal
/// </summary>
/// <param name="normal3F">������</param>
/// <returns>Normal</returns>
Normal toNormal(const Normal3F& normal3F)
{
	const Normal& normal = Normal(normal3F.NX, normal3F.NY, normal3F.NZ);

	return normal;
}

/// <summary>
/// ��������ӳ�����
/// </summary>
/// <param name="normal3Fs">��������</param>
/// <param name="length">����</param>
/// <returns>����</returns>
PointCloud<Normal> toPointCloud(Normal3F normal3Fs[], const int& length)
{
	PointCloud<Normal> normals;
	for (int i = 0; i < length; i++)
	{
		const Normal3F& normal3F = normal3Fs[i];
		const Normal& normal = Normal(normal3F.NX, normal3F.NY, normal3F.NZ);
		normals.push_back(normal);
	}

	return normals;
}

/// <summary>
/// ����ӳ�䷨�������ṹ��
/// </summary>
/// <param name="pointCloud">����</param>
/// <returns>���������ṹ��</returns>
Normal3Fs* toNormal3Fs(const PointCloud<Normal>& pointCloud)
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
