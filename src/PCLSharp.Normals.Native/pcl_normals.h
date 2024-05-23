#pragma once
#define EXPORT_C extern "C" __declspec(dllexport)
#define CALLING_MODE _cdecl
#include <point3f.h>
#include <normal3fs.h>

/// <summary>
/// ���㷨����
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="k">������������</param>
/// <returns>��������</returns>
EXPORT_C Normal3Fs* CALLING_MODE estimateNormalsByK(Point3F points[], int length, int k);

/// <summary>
/// ���㷨����
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="radius">�����뾶</param>
/// <returns>��������</returns>
EXPORT_C Normal3Fs* CALLING_MODE estimateNormalsByRadius(Point3F points[], int length, float radius);

/// <summary>
/// ���㷨���� (OMP)
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="k">������������</param>
/// <returns>��������</returns>
EXPORT_C Normal3Fs* CALLING_MODE estimateNormalsByKP(Point3F points[], int length, int k);

/// <summary>
/// ���㷨���� (OMP)
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="radius">�����뾶</param>
/// <returns>��������</returns>
EXPORT_C Normal3Fs* CALLING_MODE estimateNormalsByRadiusP(Point3F points[], int length, float radius);

/// <summary>
/// ��������
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <returns>���������</returns>
EXPORT_C Point3F* CALLING_MODE estimateCentroid(Point3F points[], int length);

/// <summary>
/// �ͷ���Դ
/// </summary>
/// <param name="pointer">ָ��</param>
EXPORT_C void CALLING_MODE dispose(const int* pointer);
