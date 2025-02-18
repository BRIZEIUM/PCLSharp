#include "primitives_map.h"
using namespace std;
using namespace pcl;

/// <summary>
/// 坐标点映射PointXYZ
/// </summary>
/// <param name="point3F">坐标点</param>
/// <returns>PointXYZ</returns>
PointXYZ pclsharp::toPointXYZ(const Point3F& point3F)
{
	const PointXYZ pointXYZ = PointXYZ(point3F.X, point3F.Y, point3F.Z);

	return pointXYZ;
}

/// <summary>
/// 坐标点集映射点云
/// </summary>
/// <param name="point3Fs">坐标点集</param>
/// <param name="length">长度</param>
/// <returns>PointXYZ点云</returns>
PointCloud<PointXYZ>::Ptr pclsharp::toPointCloud(Point3F point3Fs[], const int& length)
{
	const PointCloud<PointXYZ>::Ptr& pointCloud = std::make_shared<PointCloud<PointXYZ>>();
	for (int i = 0; i < length; i++)
	{
		const Point3F& point3F = point3Fs[i];
		const PointXYZ& pointXYZ = PointXYZ(point3F.X, point3F.Y, point3F.Z);
		pointCloud->push_back(pointXYZ);
	}

	return pointCloud;
}

/// <summary>
/// 点云映射坐标点集
/// </summary>
/// <param name="pointCloud">PointXYZ点云</param>
/// <returns>坐标点集</returns>
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
/// 法向量集映射点云
/// </summary>
/// <param name="normal3Fs">法向量集</param>
/// <param name="length">长度</param>
/// <returns>Normal点云</returns>
PointCloud<Normal>::Ptr pclsharp::toPointCloud(Normal3F normal3Fs[], const int& length)
{
	const PointCloud<Normal>::Ptr& pointCloud = std::make_shared<PointCloud<Normal>>();
	for (int i = 0; i < length; i++)
	{
		const Normal3F& normal3F = normal3Fs[i];
		const Normal& normal = Normal(normal3F.NX, normal3F.NY, normal3F.NZ);
		pointCloud->push_back(normal);
	}

	return pointCloud;
}

/// <summary>
/// 点云映射法向量集
/// </summary>
/// <param name="pointCloud">Normal点云</param>
/// <returns>法向量集</returns>
Normal3Fs* pclsharp::toNormal3Fs(const PointCloud<Normal>& pointCloud)
{
	const size_t& length = pointCloud.size();
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
/// 坐标点法向量集映射点云
/// </summary>
/// <param name="point3Normal3s">坐标点法向量集</param>
/// <param name="length">长度</param>
/// <returns>PointNormal点云</returns>
PointCloud<PointNormal>::Ptr pclsharp::toPointCloud(Point3Normal3 point3Normal3s[], const int& length)
{
	const PointCloud<PointNormal>::Ptr& pointCloud = std::make_shared<PointCloud<PointNormal>>();
	for (int i = 0; i < length; i++)
	{
		const Point3Normal3& point3Normal3 = point3Normal3s[i];
		const PointNormal& pointNormal = PointNormal(point3Normal3.X, point3Normal3.Y, point3Normal3.Z, point3Normal3.NX, point3Normal3.NY, point3Normal3.NZ);
		pointCloud->push_back(pointNormal);
	}

	return pointCloud;
}

/// <summary>
/// 点云映射坐标点法向量集
/// </summary>
/// <param name="pointCloud">PointNormal点云</param>
/// <returns>坐标点法向量集</returns>
Point3Normal3s* pclsharp::toPoint3Normal3s(const PointCloud<PointNormal>& pointCloud)
{
	const size_t& length = pointCloud.size();
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
/// 坐标点颜色集映射点云
/// </summary>
/// <param name="point3Color4s">坐标点颜色集</param>
/// <param name="length">长度</param>
/// <returns>PointXYZRGB点云</returns>
PointCloud<PointXYZRGB>::Ptr pclsharp::toPointCloudRGB(Point3Color4 point3Color4s[], const int& length)
{
	const PointCloud<PointXYZRGB>::Ptr& pointCloud = std::make_shared<PointCloud<PointXYZRGB>>();
	for (int i = 0; i < length; i++)
	{
		const Point3Color4& point3Color4 = point3Color4s[i];
		const PointXYZRGB& pointColor = PointXYZRGB(point3Color4.X, point3Color4.Y, point3Color4.Z, point3Color4.R, point3Color4.G, point3Color4.B);
		pointCloud->push_back(pointColor);
	}

	return pointCloud;
}

/// <summary>
/// 坐标点颜色集映射点云
/// </summary>
/// <param name="point3Color4s">坐标点颜色集</param>
/// <param name="length">长度</param>
/// <returns>PointXYZRGBA点云</returns>
PointCloud<PointXYZRGBA>::Ptr pclsharp::toPointCloudRGBA(Point3Color4 point3Color4s[], const int& length)
{
	const PointCloud<PointXYZRGBA>::Ptr& pointCloud = std::make_shared<PointCloud<PointXYZRGBA>>();
	for (int i = 0; i < length; i++)
	{
		const Point3Color4& point3Color4 = point3Color4s[i];
		const PointXYZRGBA& pointColor = PointXYZRGBA(point3Color4.X, point3Color4.Y, point3Color4.Z, point3Color4.R, point3Color4.G, point3Color4.B, point3Color4.A);
		pointCloud->push_back(pointColor);
	}

	return pointCloud;
}

/// <summary>
/// 点云映射坐标点颜色集
/// </summary>
/// <param name="pointCloud">PointXYZRGB点云</param>
/// <returns>坐标点颜色集</returns>
Point3Color4s* pclsharp::toPoint3Color4s(const pcl::PointCloud<pcl::PointXYZRGB>& pointCloud)
{
	const size_t& length = pointCloud.size();
	Point3Color4* pointColors = new Point3Color4[length];
	for (int i = 0; i < length; i++)
	{
		const PointXYZRGB& pointColor = pointCloud.points[i];
		pointColors[i] = Point3Color4(pointColor.x, pointColor.y, pointColor.z, pointColor.r, pointColor.g, pointColor.b, 255);
	}

	Point3Color4s* point3Color4s = new Point3Color4s(pointColors, static_cast<int>(length));

	return point3Color4s;
}

/// <summary>
/// 点云映射坐标点颜色集
/// </summary>
/// <param name="pointCloud">PointXYZRGBA点云</param>
/// <returns>坐标点颜色集</returns>
Point3Color4s* pclsharp::toPoint3Color4s(const PointCloud<PointXYZRGBA>& pointCloud)
{
	const size_t& length = pointCloud.size();
	Point3Color4* pointColors = new Point3Color4[length];
	for (int i = 0; i < length; i++)
	{
		const PointXYZRGBA& pointColor = pointCloud.points[i];
		pointColors[i] = Point3Color4(pointColor.x, pointColor.y, pointColor.z, pointColor.r, pointColor.g, pointColor.b, pointColor.a);
	}

	Point3Color4s* point3Color4s = new Point3Color4s(pointColors, static_cast<int>(length));

	return point3Color4s;
}
