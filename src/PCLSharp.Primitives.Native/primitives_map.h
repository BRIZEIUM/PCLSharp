#pragma once
#define EXPORT_CPP extern __declspec(dllexport)
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "point3f.h"
#include "point3fs.h"

namespace pclsharp
{
	/// <summary>
	/// �����ӳ��PointXYZ
	/// </summary>
	/// <param name="point3F">3D�����</param>
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
}
