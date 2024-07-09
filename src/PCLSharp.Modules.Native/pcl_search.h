#pragma once
#ifdef _WIN32
#define EXPORT_C extern "C" __declspec(dllexport)
#define CALLING_MODE _cdecl
#elif __linux__
#define EXPORT_C extern "C"
#define CALLING_MODE __attribute__((__stdcall__))
#endif
#include <point3f.h>
#include <point3fs.h>

/// <summary>
/// K��������
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="referencePoint">�ο������</param>
/// <param name="k">��������</param>
/// <returns>����㼯</returns>
EXPORT_C Point3Fs* CALLING_MODE kSearch(Point3F points[], int length, Point3F referencePoint, int k);

/// <summary>
/// �뾶����
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="referencePoint">�ο������</param>
/// <param name="radius">�����뾶</param>
/// <returns>����㼯</returns>
EXPORT_C Point3Fs* CALLING_MODE radiusSearch(Point3F points[], int length, Point3F referencePoint, float radius);

/// <summary>
/// �˲�������
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="referencePoint">�ο������</param>
/// <param name="resolution">�ֱ���</param>
/// <returns>����㼯</returns>
EXPORT_C Point3Fs* CALLING_MODE octreeSearch(Point3F points[], int length, Point3F referencePoint, float resolution);
