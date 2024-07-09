#pragma once
#ifdef _WIN32
#define EXPORT_C extern "C" __declspec(dllexport)
#define CALLING_MODE _cdecl
#elif __linux__
#define EXPORT_C extern "C"
#define CALLING_MODE __attribute__((__cdecl__))
#endif
#include <point3f.h>
#include <point3fs.h>
#include <pose.h>

/// <summary>
/// ��������
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <returns>���������</returns>
EXPORT_C Point3F* CALLING_MODE estimateCentroid(Point3F points[], int length);

/// <summary>
/// ����任
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="pose">λ��</param>
/// <returns>�任�����</returns>
EXPORT_C Point3Fs* CALLING_MODE affineTransform(Point3F points[], int length, Pose pose);

/// <summary>
/// ����任
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="matrixArray">��������(����: 16)</param>
/// <returns>�任�����</returns>
EXPORT_C Point3Fs* CALLING_MODE matrixTransform(Point3F points[], int length, float matrixArray[]);

/// <summary>
/// �ϲ�����㷨����
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="normal3Fs">��������</param>
/// <param name="length">�㼯����</param>
/// <returns>�㼯</returns>
EXPORT_C Point3Normal3s* CALLING_MODE mergePointsNormals(Point3F points[], Normal3F normal3Fs[], int length);

/// <summary>
/// ���������
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="minPoint">��С�����</param>
/// <param name="maxPoint">��������</param>
/// <param name="negative">true: ����/false: ����</param>
/// <returns>���ú����</returns>
EXPORT_C Point3Fs* CALLING_MODE cropBox(Point3F points[], int length, Point3F minPoint, Point3F maxPoint, bool negative);

/// <summary>
/// ͹������
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="contourPoints">�����㼯</param>
/// <param name="contourLength">�����㼯����</param>
/// <param name="dimensionsCount">ά����</param>
/// <returns>���ú����</returns>
EXPORT_C Point3Fs* CALLING_MODE cropConvexHull(Point3F points[], int length, Point3F contourPoints[], int contourLength, int dimensionsCount);

/// <summary>
/// Ͷ��ƽ��
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="a">ƽ�淽��ϵ��a</param>
/// <param name="b">ƽ�淽��ϵ��b</param>
/// <param name="c">ƽ�淽��ϵ��c</param>
/// <param name="d">ƽ�淽��ϵ��d</param>
/// <returns>Ͷ������</returns>
/// <remarks>ƽ�淽��: ax + by +cz + d = 0</remarks>
EXPORT_C Point3Fs* CALLING_MODE projectPlane(Point3F points[], int length, float a, float b, float c, float d);

/// <summary>
/// ��ȡ�߿�
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <returns>�߿����</returns>
EXPORT_C Point3Fs* CALLING_MODE extractBorder(Point3F points[], int length);

/// <summary>
/// ��ȡ�߽�
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="normalK">������K</param>
/// <param name="featureRadius">�����뾶</param>
/// <param name="angleThreshold">�Ƕ���ֵ</param>
/// <param name="threadsCount">�߳���</param>
/// <returns>�߽����</returns>
EXPORT_C Point3Fs* CALLING_MODE extractBoundary(Point3F points[], int length, int normalK, float featureRadius, float angleThreshold, int threadsCount);
