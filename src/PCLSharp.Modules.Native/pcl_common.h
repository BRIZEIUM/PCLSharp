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
/// Ͷ����
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="a">�߷���ϵ��a</param>
/// <param name="b">�߷���ϵ��b</param>
/// <param name="c">�߷���ϵ��c</param>
/// <param name="d">�߷���ϵ��d</param>
/// <param name="e">�߷���ϵ��e</param>
/// <param name="f">�߷���ϵ��f</param>
/// <returns>Ͷ������</returns>
/// <remarks>�߷���: (x, y, z) = (a, b, c) + t(d, e, f)</remarks>
EXPORT_C Point3Fs* CALLING_MODE projectLine(Point3F points[], int length, float a, float b, float c, float d, float e, float f);

/// <summary>
/// Ͷ��2DԲ
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="x">Բ��x����</param>
/// <param name="y">Բ��y����</param>
/// <param name="radius">�뾶</param>
/// <returns>Ͷ������</returns>
EXPORT_C Point3Fs* CALLING_MODE projectCircle2D(Point3F points[], int length, float x, float y, float radius);

/// <summary>
/// Ͷ��3DԲ
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="x">Բ��x����</param>
/// <param name="y">Բ��y����</param>
/// <param name="z">Բ��z����</param>
/// <param name="radius">�뾶</param>
/// <returns>Ͷ������</returns>
EXPORT_C Point3Fs* CALLING_MODE projectCircle3D(Point3F points[], int length, float x, float y, float z, float radius);

/// <summary>
/// Ͷ����
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="x">����x����</param>
/// <param name="y">����y����</param>
/// <param name="z">����z����</param>
/// <param name="radius">�뾶</param>
/// <returns>Ͷ������</returns>
EXPORT_C Point3Fs* CALLING_MODE projectSphere(Point3F points[], int length, float x, float y, float z, float radius);

/// <summary>
/// Ͷ��Բ��
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="x">Բ�����������x����</param>
/// <param name="y">Բ�����������y����</param>
/// <param name="z">Բ�����������z����</param>
/// <param name="dx">Բ�������߷���x����</param>
/// <param name="dy">Բ�������߷���y����</param>
/// <param name="dz">Բ�������߷���z����</param>
/// <param name="radius">�뾶</param>
/// <returns>Ͷ������</returns>
EXPORT_C Point3Fs* CALLING_MODE projectCylinder(Point3F points[], int length, float x, float y, float z, float dx, float dy, float dz, float radius);

/// <summary>
/// Ͷ��Բ׶
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="x">Բ׶����x����</param>
/// <param name="y">Բ׶����y����</param>
/// <param name="z">Բ׶����z����</param>
/// <param name="dx">Բ׶����x����</param>
/// <param name="dy">Բ׶����y����</param>
/// <param name="dz">Բ׶����z����</param>
/// <param name="angle">Բ׶�Ƕ�</param>
/// <returns>Ͷ������</returns>
EXPORT_C Point3Fs* CALLING_MODE projectCone(Point3F points[], int length, float x, float y, float z, float dx, float dy, float dz, float angle);

/// <summary>
/// Ͷ��Բ��
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="x">Բ������x����</param>
/// <param name="y">Բ������y����</param>
/// <param name="z">Բ������z����</param>
/// <param name="dx">Բ��������x����</param>
/// <param name="dy">Բ��������y����</param>
/// <param name="dz">Բ��������z����</param>
/// <param name="radius">Բ���뾶</param>
/// <param name="tube_radius">Բ���ܰ뾶</param>
/// <returns>Ͷ������</returns>
EXPORT_C Point3Fs* CALLING_MODE projectTorus(Point3F points[], int length, float x, float y, float z, float dx, float dy, float dz, float radius, float tube_radius);

// ����ģ�͵ĺ������԰��������ģ��������Ƶı�д������ģ�͵ľ���������е�����
/// <summary>
/// Ͷ��ƽ����
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="x">ƽ���߷���x����</param>
/// <param name="y">ƽ���߷���y����</param>
/// <param name="z">ƽ���߷���z����</param>
/// <returns>Ͷ������</returns>
EXPORT_C Point3Fs* CALLING_MODE projectParallelLine(Point3F points[], int length, float x, float y, float z);

/// <summary>
/// Ͷ�䴹ֱƽ��
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="x">������x����</param>
/// <param name="y">������y����</param>
/// <param name="z">������z����</param>
/// <returns>Ͷ������</returns>
EXPORT_C Point3Fs* CALLING_MODE projectPerpendicularPlane(Point3F points[], int length, float x, float y, float z);

/// <summary>
/// Ͷ��ƽ���߼���
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="x">ƽ���߷���x����</param>
/// <param name="y">ƽ���߷���y����</param>
/// <param name="z">ƽ���߷���z����</param>
/// <returns>Ͷ������</returns>
EXPORT_C Point3Fs* CALLING_MODE projectParallelLines(Point3F points[], int length, float x, float y, float z);

/// <summary>
/// Ͷ�䷨��ƽ��
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="x">������x����</param>
/// <param name="y">������y����</param>
/// <param name="z">������z����</param>
/// <param name="nx">������x����</param>
/// <param name="ny">������y����</param>
/// <param name="nz">������z����</param>
/// <returns>Ͷ������</returns>
EXPORT_C Point3Fs* CALLING_MODE projectNormalPlane(Point3F points[], int length, float x, float y, float z, float nx, float ny, float nz);

/// <summary>
/// Ͷ�䷨����
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="x">����x����</param>
/// <param name="y">����y����</param>
/// <param name="z">����z����</param>
/// <param name="radius">�뾶</param>
/// <param name="nx">������x����</param>
/// <param name="ny">������y����</param>
/// <param name="nz">������z����</param>
/// <returns>Ͷ������</returns>
EXPORT_C Point3Fs* CALLING_MODE projectNormalSphere(Point3F points[], int length, float x, float y, float z, float radius, float nx, float ny, float nz);

/// <summary>
/// Ͷ��ע��ģ��
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="tx">ƽ������x����</param>
/// <param name="ty">ƽ������y����</param>
/// <param="tz">ƽ������z����</param>
/// <param name="qx">��Ԫ��x����</param>
/// <param name="qy">��Ԫ��y����</param>
/// <param name="qz">��Ԫ��z����</param>
/// <param name="qw">��Ԫ��w����</param>
/// <returns>Ͷ������</returns>
EXPORT_C Point3Fs* CALLING_MODE projectRegistration(Point3F points[], int length, float tx, float ty, float tz, float qx, float qy, float qz, float qw);

/// <summary>
/// Ͷ��2Dע��ģ��
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="tx">ƽ������x����</param>
/// <param name="ty">ƽ������y����</param>
/// <param name="tz">ƽ������z����</param>
/// <param name="qx">��Ԫ��x����</param>
/// <param name="qy">��Ԫ��y����</param>
/// <param name="qz">��Ԫ��z����</param>
/// <param name="qw">��Ԫ��w����</param>
/// <returns>Ͷ������</returns>
EXPORT_C Point3Fs* CALLING_MODE projectRegistration2D(Point3F points[], int length, float tx, float ty, float tz, float qx, float qy, float qz, float qw);

/// <summary>
/// Ͷ��ƽ��ƽ��
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="x">������x����</param>
/// <param name="y">������y����</param>
/// <param name="z">������z����</param>
/// <returns>Ͷ������</returns>
EXPORT_C Point3Fs* CALLING_MODE projectParallelPlane(Point3F points[], int length, float x, float y, float z);

/// <summary>
/// Ͷ��ƽ�з���ƽ��
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="x">������x����</param>
/// <param name="y">������y����</param>
/// <param name="z">������z����</param>
/// <returns>Ͷ������</returns>
EXPORT_C Point3Fs* CALLING_MODE projectNormalParallelPlane(Point3F points[], int length, float x, float y, float z);

/// <summary>
/// Ͷ���״��
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="x1">��״�����x����</param>
/// <param name="y1">��״�����y����</param>
/// <param name="z1">��״�����z����</param>
/// <param name="x2">��״���յ�x����</param>
/// <param name="y2">��״���յ�y����</param>
/// <param name="z2">��״���յ�z����</param>
/// <returns>Ͷ������</returns>
EXPORT_C Point3Fs* CALLING_MODE projectStick(Point3F points[], int length, float x1, float y1, float z1, float x2, float y2, float z2);

/// <summary>
/// Ͷ��3D��Բ
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="x">��Բ����x����</param>
/// <param name="y">��Բ����y����</param>
/// <param name="z">��Բ����z����</param>
/// <param name="a">����뾶</param>
/// <param name="b">����뾶</param>
/// <param name="c">��Բ������x����</param>
/// <param name="d">��Բ������y����</param>
/// <param name="e">��Բ������z����</param>
/// <returns>Ͷ������</returns>
EXPORT_C Point3Fs* CALLING_MODE projectEllipse3D(Point3F points[], int length, float x, float y, float z, float a, float b, float c, float d, float e);

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
