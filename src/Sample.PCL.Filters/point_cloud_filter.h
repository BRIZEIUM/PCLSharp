#pragma once
#define EXPORT extern "C" __declspec(dllexport)
#define CALLING_MODE _cdecl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "point3f.h"
#include "point_array.h"

/// <summary>
/// ����ֱͨ�˲�
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="axis">����������</param>
/// <param name="limitMin">���˷�Χ��Сֵ</param>
/// <param name="limixMax">���˷�Χ���ֵ</param>
/// <returns>���˺�㼯</returns>
EXPORT PointArray* CALLING_MODE applyPassThrogh(Point3F points[], int length, const char* axis, float limitMin, float limixMax);

/// <summary>
/// ���þ��Ȳ���
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="radius">�����뾶</param>
/// <returns>���˺�㼯</returns>
EXPORT PointArray* CALLING_MODE applyUniformSampling(Point3F points[], int length, float radius);

/// <summary>
/// �������ؽ�����
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="leafSize">Ҷ�ߴ�</param>
/// <returns>���˺�㼯</returns>
EXPORT PointArray* CALLING_MODE applyVoxelGrid(Point3F points[], int length, float leafSize);

/// <summary>
/// ������Ⱥ���Ƴ�
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="meanK">ƽ��������Ƶ�����ھӵ�����</param>
/// <param name="stddevMult">��׼����ֵϵ��</param>
/// <returns>���˺�㼯</returns>
EXPORT PointArray* CALLING_MODE applyOutlierRemoval(Point3F points[], int length, int meanK, float stddevMult);

/// <summary>
/// �ͷ���Դ
/// </summary>
/// <param name="pointer">ָ��</param>
EXPORT void CALLING_MODE dispose(const PointArray* pointer);
