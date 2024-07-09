#pragma once
#ifdef _WIN32
#define EXPORT_C extern "C" __declspec(dllexport)
#define CALLING_MODE _cdecl
#elif __linux__
#define EXPORT_C extern "C"
#define CALLING_MODE __attribute__((__stdcall__))
#endif
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
