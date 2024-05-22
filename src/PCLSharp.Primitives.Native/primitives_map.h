#pragma once
#define EXPORT_CPP extern __declspec(dllexport)
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "point3f.h"
#include "point3fs.h"
#include "normal3f.h"
#include "normal3fs.h"

namespace pclsharp
{
	/// <summary>
	/// �����ӳ��PointXYZ
	/// </summary>
	/// <param name="point3F">�����</param>
	/// <returns>PointXYZ</returns>
	EXPORT_CPP pcl::PointXYZ toPointXYZ(const Point3F& point3F);

	/// <summary>
	/// ����㼯ӳ�����
	/// </summary>
	/// <param name="point3Fs">����㼯</param>
	/// <param name="length">����</param>
	/// <returns>����</returns>
	EXPORT_CPP pcl::PointCloud<pcl::PointXYZ> toPointCloud(Point3F point3Fs[], const int& length);

	/// <summary>
	/// ����ӳ������㼯�ṹ��
	/// </summary>
	/// <param name="pointCloud">����</param>
	/// <returns>����㼯�ṹ��</returns>
	EXPORT_CPP Point3Fs* toPoint3Fs(const pcl::PointCloud<pcl::PointXYZ>& pointCloud);

	/// <summary>
	/// ������ӳ��Normal
	/// </summary>
	/// <param name="normal3F">������</param>
	/// <returns>Normal</returns>
	EXPORT_CPP pcl::Normal toNormal(const Normal3F& normal3F);

	/// <summary>
	/// ��������ӳ�����
	/// </summary>
	/// <param name="normal3Fs">��������</param>
	/// <param name="length">����</param>
	/// <returns>����</returns>
	EXPORT_CPP pcl::PointCloud<pcl::Normal> toPointCloud(Normal3F normal3Fs[], const int& length);

	/// <summary>
	/// ����ӳ�䷨�������ṹ��
	/// </summary>
	/// <param name="pointCloud">����</param>
	/// <returns>���������ṹ��</returns>
	EXPORT_CPP Normal3Fs* toNormal3Fs(const pcl::PointCloud<pcl::Normal>& pointCloud);
}
