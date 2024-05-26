#pragma once
#define EXPORT_CPP extern __declspec(dllexport)
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "point3f.h"
#include "point3fs.h"
#include "point3color4.h"
#include "point3color4s.h"
#include "point3normal3.h"
#include "point3normal3s.h"
#include "normal3f.h"
#include "normal3fs.h"

namespace pclsharp
{
	/// <summary>
	/// ����㼯ӳ�����
	/// </summary>
	/// <param name="point3Fs">����㼯</param>
	/// <param name="length">����</param>
	/// <returns>PointXYZ����</returns>
	EXPORT_CPP pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud(Point3F point3Fs[], const int& length);

	/// <summary>
	/// ����ӳ������㼯
	/// </summary>
	/// <param name="pointCloud">PointXYZ����</param>
	/// <returns>����㼯</returns>
	EXPORT_CPP Point3Fs* toPoint3Fs(const pcl::PointCloud<pcl::PointXYZ>& pointCloud);

	/// <summary>
	/// ��������ӳ�����
	/// </summary>
	/// <param name="normal3Fs">��������</param>
	/// <param name="length">����</param>
	/// <returns>Normal����</returns>
	EXPORT_CPP pcl::PointCloud<pcl::Normal>::Ptr toPointCloud(Normal3F normal3Fs[], const int& length);

	/// <summary>
	/// ����ӳ�䷨������
	/// </summary>
	/// <param name="pointCloud">Normal����</param>
	/// <returns>��������</returns>
	EXPORT_CPP Normal3Fs* toNormal3Fs(const pcl::PointCloud<pcl::Normal>& pointCloud);

	/// <summary>
	/// ����㷨������ӳ�����
	/// </summary>
	/// <param name="point3Normal3s">����㷨������</param>
	/// <param name="length">����</param>
	/// <returns>PointNormal����</returns>
	EXPORT_CPP pcl::PointCloud<pcl::PointNormal>::Ptr toPointCloud(Point3Normal3 point3Normal3s[], const int& length);

	/// <summary>
	/// ����ӳ������㷨������
	/// </summary>
	/// <param name="pointCloud">PointNormal����</param>
	/// <returns>����㷨������</returns>
	EXPORT_CPP Point3Normal3s* toPoint3Normal3s(const pcl::PointCloud<pcl::PointNormal>& pointCloud);

	/// <summary>
	/// �������ɫ��ӳ�����
	/// </summary>
	/// <param name="point3Color4s">�������ɫ��</param>
	/// <param name="length">����</param>
	/// <returns>PointXYZRGBA����</returns>
	EXPORT_CPP pcl::PointCloud<pcl::PointXYZRGBA>::Ptr toPointCloud(Point3Color4 point3Color4s[], const int& length);

	/// <summary>
	/// ����ӳ���������ɫ��
	/// </summary>
	/// <param name="pointCloud">PointXYZRGBA����</param>
	/// <returns>�������ɫ��</returns>
	EXPORT_CPP Point3Color4s* toPoint3Color4s(const pcl::PointCloud<pcl::PointXYZRGBA>& pointCloud);
}
