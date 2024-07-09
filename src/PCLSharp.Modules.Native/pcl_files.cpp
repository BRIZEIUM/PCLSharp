#include <format>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <primitives_map.h>
#include "pcl_files.h"
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
	throw invalid_argument(message.c_str());
}

/// <summary>
/// ���ط�����PCD�ļ�
/// </summary>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
Point3Normal3s* loadNormalPCD(const char* filePath)
{
	PointCloud<PointNormal> cloud;
	const int& loadStatus = pcl::io::loadPCDFile(filePath, cloud);
	if (loadStatus == 0)
	{
		Point3Normal3s* point3Normal3s = pclsharp::toPoint3Normal3s(cloud);
		return point3Normal3s;
	}

	const string message = std::format("����\"{}\"ʱ����", filePath);
	throw invalid_argument(message.c_str());
}

/// <summary>
/// ������ɫPCD�ļ�
/// </summary>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
Point3Color4s* loadColorPCD(const char* filePath)
{
	PointCloud<PointXYZRGBA> cloud;
	const int& loadStatus = pcl::io::loadPCDFile(filePath, cloud);
	if (loadStatus == 0)
	{
		Point3Color4s* point3Color4s = pclsharp::toPoint3Color4s(cloud);
		return point3Color4s;
	}

	const string message = std::format("����\"{}\"ʱ����", filePath);
	throw invalid_argument(message.c_str());
}

/// <summary>
/// ����PLY�ļ�
/// </summary>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
Point3Fs* loadPLY(const char* filePath)
{
	PointCloud<PointXYZ> cloud;
	const int& loadStatus = pcl::io::loadPLYFile(filePath, cloud);
	if (loadStatus == 0)
	{
		Point3Fs* point3Fs = pclsharp::toPoint3Fs(cloud);
		return point3Fs;
	}

	const string message = std::format("����\"{}\"ʱ����", filePath);
	throw invalid_argument(message.c_str());
}

/// <summary>
/// ���ط�����PLY�ļ�
/// </summary>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
Point3Normal3s* loadNormalPLY(const char* filePath)
{
	PointCloud<PointNormal> cloud;
	const int& loadStatus = pcl::io::loadPLYFile(filePath, cloud);
	if (loadStatus == 0)
	{
		Point3Normal3s* point3Normal3s = pclsharp::toPoint3Normal3s(cloud);
		return point3Normal3s;
	}

	const string message = std::format("����\"{}\"ʱ����", filePath);
	throw invalid_argument(message.c_str());
}

/// <summary>
/// ������ɫPLY�ļ�
/// </summary>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
Point3Color4s* loadColorPLY(const char* filePath)
{
	PointCloud<PointXYZRGBA> cloud;
	const int& loadStatus = pcl::io::loadPLYFile(filePath, cloud);
	if (loadStatus == 0)
	{
		Point3Color4s* point3Color4s = pclsharp::toPoint3Color4s(cloud);
		return point3Color4s;
	}

	const string message = std::format("����\"{}\"ʱ����", filePath);
	throw invalid_argument(message.c_str());
}

/// <summary>
/// ����OBJ�ļ�
/// </summary>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
Point3Fs* loadOBJ(const char* filePath)
{
	PointCloud<PointXYZ> cloud;
	const int& loadStatus = pcl::io::loadOBJFile(filePath, cloud);
	if (loadStatus == 0)
	{
		Point3Fs* point3Fs = pclsharp::toPoint3Fs(cloud);
		return point3Fs;
	}

	const string message = std::format("����\"{}\"ʱ����", filePath);
	throw invalid_argument(message.c_str());
}

/// <summary>
/// ���ط�����OBJ�ļ�
/// </summary>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
Point3Normal3s* loadNormalOBJ(const char* filePath)
{
	PointCloud<PointNormal> cloud;
	const int& loadStatus = pcl::io::loadOBJFile(filePath, cloud);
	if (loadStatus == 0)
	{
		Point3Normal3s* point3Normal3s = pclsharp::toPoint3Normal3s(cloud);
		return point3Normal3s;
	}

	const string message = std::format("����\"{}\"ʱ����", filePath);
	throw invalid_argument(message.c_str());
}

/// <summary>
/// ������ɫOBJ�ļ�
/// </summary>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
Point3Color4s* loadColorOBJ(const char* filePath)
{
	PointCloud<PointXYZRGBA> cloud;
	const int& loadStatus = pcl::io::loadOBJFile(filePath, cloud);
	if (loadStatus == 0)
	{
		Point3Color4s* point3Color4s = pclsharp::toPoint3Color4s(cloud);
		return point3Color4s;
	}

	const string message = std::format("����\"{}\"ʱ����", filePath);
	throw invalid_argument(message.c_str());
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
	const PointCloud<PointXYZ>::Ptr& cloud = pclsharp::toPointCloud(points, length);
	const int& saveStatus = pcl::io::savePCDFileASCII(filePath, *cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("����\"{}\"ʱ����", filePath);
		throw invalid_argument(message.c_str());
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
	const PointCloud<PointXYZ>::Ptr& cloud = pclsharp::toPointCloud(points, length);
	const int& saveStatus = pcl::io::savePCDFileBinaryCompressed(filePath, *cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("����\"{}\"ʱ����", filePath);
		throw invalid_argument(message.c_str());
	}
}

/// <summary>
/// ���淨����PCD�ı��ļ�
/// </summary>
/// <param name="pointNormals">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
void saveNormalTextPCD(Point3Normal3 pointNormals[], int length, const char* filePath)
{
	const PointCloud<PointNormal>::Ptr& cloud = pclsharp::toPointCloud(pointNormals, length);
	const int& saveStatus = pcl::io::savePCDFileASCII(filePath, *cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("����\"{}\"ʱ����", filePath);
		throw invalid_argument(message.c_str());
	}
}

/// <summary>
/// ���淨����PCD�������ļ�
/// </summary>
/// <param name="pointNormals">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
void saveNormalBinaryPCD(Point3Normal3 pointNormals[], int length, const char* filePath)
{
	const PointCloud<PointNormal>::Ptr& cloud = pclsharp::toPointCloud(pointNormals, length);
	const int& saveStatus = pcl::io::savePCDFileBinaryCompressed(filePath, *cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("����\"{}\"ʱ����", filePath);
		throw invalid_argument(message.c_str());
	}
}

/// <summary>
/// ������ɫPCD�ı��ļ�
/// </summary>
/// <param name="pointColors">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
void saveColorTextPCD(Point3Color4 pointColors[], int length, const char* filePath)
{
	const PointCloud<PointXYZRGBA>::Ptr& cloud = pclsharp::toPointCloudRGBA(pointColors, length);
	const int& saveStatus = pcl::io::savePCDFileASCII(filePath, *cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("����\"{}\"ʱ����", filePath);
		throw invalid_argument(message.c_str());
	}
}

/// <summary>
/// ������ɫPCD�������ļ�
/// </summary>
/// <param name="pointColors">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
void saveColorBinaryPCD(Point3Color4 pointColors[], int length, const char* filePath)
{
	const PointCloud<PointXYZRGBA>::Ptr& cloud = pclsharp::toPointCloudRGBA(pointColors, length);
	const int& saveStatus = pcl::io::savePCDFileBinaryCompressed(filePath, *cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("����\"{}\"ʱ����", filePath);
		throw invalid_argument(message.c_str());
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
	const PointCloud<PointXYZ>::Ptr& cloud = pclsharp::toPointCloud(points, length);
	const int& saveStatus = pcl::io::savePLYFileASCII(filePath, *cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("����\"{}\"ʱ����", filePath);
		throw invalid_argument(message.c_str());
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
	const PointCloud<PointXYZ>::Ptr& cloud = pclsharp::toPointCloud(points, length);
	const int& saveStatus = pcl::io::savePLYFileBinary(filePath, *cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("����\"{}\"ʱ����", filePath);
		throw invalid_argument(message.c_str());
	}
}

/// <summary>
/// ���淨����PLY�ı��ļ�
/// </summary>
/// <param name="pointNormals">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
void saveNormalTextPLY(Point3Normal3 pointNormals[], int length, const char* filePath)
{
	const PointCloud<PointNormal>::Ptr& cloud = pclsharp::toPointCloud(pointNormals, length);
	const int& saveStatus = pcl::io::savePLYFileASCII(filePath, *cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("����\"{}\"ʱ����", filePath);
		throw invalid_argument(message.c_str());
	}
}

/// <summary>
/// ���淨����PLY�������ļ�
/// </summary>
/// <param name="pointNormals">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
void saveNormalBinaryPLY(Point3Normal3 pointNormals[], int length, const char* filePath)
{
	const PointCloud<PointNormal>::Ptr& cloud = pclsharp::toPointCloud(pointNormals, length);
	const int& saveStatus = pcl::io::savePLYFileBinary(filePath, *cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("����\"{}\"ʱ����", filePath);
		throw invalid_argument(message.c_str());
	}
}

/// <summary>
/// ������ɫPLY�ı��ļ�
/// </summary>
/// <param name="pointColors">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
void saveColorTextPLY(Point3Color4 pointColors[], int length, const char* filePath)
{
	const PointCloud<PointXYZRGBA>::Ptr& cloud = pclsharp::toPointCloudRGBA(pointColors, length);
	const int& saveStatus = pcl::io::savePLYFileASCII(filePath, *cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("����\"{}\"ʱ����", filePath);
		throw invalid_argument(message.c_str());
	}
}

/// <summary>
/// ������ɫPLY�������ļ�
/// </summary>
/// <param name="pointColors">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
void saveColorBinaryPLY(Point3Color4 pointColors[], int length, const char* filePath)
{
	const PointCloud<PointXYZRGBA>::Ptr& cloud = pclsharp::toPointCloudRGBA(pointColors, length);
	const int& saveStatus = pcl::io::savePLYFileBinary(filePath, *cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("����\"{}\"ʱ����", filePath);
		throw invalid_argument(message.c_str());
	}
}
