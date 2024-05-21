#include <format>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <primitives_map.h>
#include "pcl_conductor.h"
using namespace std;
using namespace pcl;

/// <summary>
/// ����PCD�ļ�
/// </summary>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
Point3Fs* loadPCD(const char* filePath)
{
	PointCloud<PointXYZ> cloud;
	const int& loadStatus = pcl::io::loadPCDFile(filePath, cloud);
	if (loadStatus == 0)
	{
		Point3Fs* point3Fs = pclsharp::toPoint3Fs(cloud);
		return point3Fs;
	}

	const string message = std::format("����\"{}\"ʱ����", filePath);
	throw exception(message.c_str());
}

/// <summary>
/// ����PLY�ļ�
/// </summary>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
Point3Fs* CALLING_MODE loadPLY(const char* filePath)
{
	PointCloud<PointXYZ> cloud;
	const int& loadStatus = pcl::io::loadPLYFile(filePath, cloud);
	if (loadStatus == 0)
	{
		Point3Fs* point3Fs = pclsharp::toPoint3Fs(cloud);
		return point3Fs;
	}

	const string message = std::format("����\"{}\"ʱ����", filePath);
	throw exception(message.c_str());
}

/// <summary>
/// ����OBJ�ļ�
/// </summary>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
Point3Fs* CALLING_MODE loadOBJ(const char* filePath)
{
	PointCloud<PointXYZ> cloud;
	const int& loadStatus = pcl::io::loadOBJFile(filePath, cloud);
	if (loadStatus == 0)
	{
		Point3Fs* point3Fs = pclsharp::toPoint3Fs(cloud);
		return point3Fs;
	}

	const string message = std::format("����\"{}\"ʱ����", filePath);
	throw exception(message.c_str());
}

/// <summary>
/// ����PCD�ı��ļ�
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
void saveTextPCD(Point3F points[], int length, const char* filePath)
{
	const PointCloud<PointXYZ>& cloud = pclsharp::toPointCloud(points, length);
	const int& saveStatus = pcl::io::savePCDFileASCII(filePath, cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("����\"{}\"ʱ����", filePath);
		throw exception(message.c_str());
	}
}

/// <summary>
/// ����PCD�������ļ�
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
void saveBinaryPCD(Point3F points[], int length, const char* filePath)
{
	const PointCloud<PointXYZ>& cloud = pclsharp::toPointCloud(points, length);
	const int& saveStatus = pcl::io::savePCDFileBinaryCompressed(filePath, cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("����\"{}\"ʱ����", filePath);
		throw exception(message.c_str());
	}
}

/// <summary>
/// ����PLY�ı��ļ�
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
void saveTextPLY(Point3F points[], int length, const char* filePath)
{
	const PointCloud<PointXYZ>& cloud = pclsharp::toPointCloud(points, length);
	const int& saveStatus = pcl::io::savePLYFileASCII(filePath, cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("����\"{}\"ʱ����", filePath);
		throw exception(message.c_str());
	}
}

/// <summary>
/// ����PLY�������ļ�
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
void saveBinaryPLY(Point3F points[], int length, const char* filePath)
{
	const PointCloud<PointXYZ>& cloud = pclsharp::toPointCloud(points, length);
	const int& saveStatus = pcl::io::savePLYFileBinary(filePath, cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("����\"{}\"ʱ����", filePath);
		throw exception(message.c_str());
	}
}

/// <summary>
/// �ͷ���Դ
/// </summary>
/// <param name="pointer">ָ��</param>
void dispose(const Point3Fs* pointer)
{
	delete pointer;
}
