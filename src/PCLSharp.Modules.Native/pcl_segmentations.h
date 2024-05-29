#pragma once
#define EXPORT_C extern "C" __declspec(dllexport)
#define CALLING_MODE _cdecl
#include <point3f.h>
#include <point3fs.h>
#include <point3color4.h>
#include <point3color4s.h>

/// <summary>
/// �ָ�ƽ��
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="optimizeCoefficients">�Ƿ��Ż�ģ��ϵ��</param>
/// <param name="probability">����</param>
/// <param name="distanceThreshold">������ֵ</param>
/// <param name="maxIterationsCount">����������</param>
/// <returns>ƽ�����</returns>
EXPORT_C Point3Fs* CALLING_MODE segmentPlane(Point3F points[], int length, bool optimizeCoefficients, float probability, float distanceThreshold, int maxIterationsCount);

/// <summary>
/// �ָ�����
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="optimizeCoefficients">�Ƿ��Ż�ģ��ϵ��</param>
/// <param name="probability">����</param>
/// <param name="distanceThreshold">������ֵ</param>
/// <param name="minRadius">������С�뾶</param>
/// <param name="maxRadius">�������뾶</param>
/// <param name="maxIterationsCount">����������</param>
/// <returns>�������</returns>
EXPORT_C Point3Fs* CALLING_MODE segmentSphere(Point3F points[], int length, bool optimizeCoefficients, float probability, float distanceThreshold, float minRadius, float maxRadius, int maxIterationsCount);

/// <summary>
/// ŷ����þ���ָ�
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="clusterTolerance">�������ݲ�</param>
/// <param name="minClusterSize">����С�ߴ�</param>
/// <param name="maxClusterSize">�����ߴ�</param>
/// <param name="clustersCount">���ƴ���</param>
/// <returns>���ƴ��б�</returns>
EXPORT_C Point3Fs** CALLING_MODE euclidClusterSegment(Point3F points[], int length, float clusterTolerance, int minClusterSize, int maxClusterSize, int& clustersCount);

/// <summary>
/// ���������ָ�
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="normalK">������K</param>
/// <param name="clusterK">��K</param>
/// <param name="smoothnessThreshold">ƽ����ֵ���Ƕȣ�</param>
/// <param name="curvatureThreshold">������ֵ</param>
/// <param name="minClusterSize">����С�ߴ�</param>
/// <param name="maxClusterSize">�����ߴ�</param>
/// <param name="threadsCount">�߳���</param>
/// <param name="clustersCount">���ƴ���</param>
/// <returns>���ƴ��б�</returns>
EXPORT_C Point3Fs** CALLING_MODE regionGrowingSegment(Point3F points[], int length, int normalK, int clusterK, float smoothnessThreshold, float curvatureThreshold, int minClusterSize, int maxClusterSize, int threadsCount, int& clustersCount);

/// <summary>
/// ����������ɫ�ָ�
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="normalK">������K</param>
/// <param name="clusterK">��K</param>
/// <param name="distanceThreshold">������ֵ</param>
/// <param name="smoothnessThreshold">ƽ����ֵ���Ƕȣ�</param>
/// <param name="curvatureThreshold">������ֵ</param>
/// <param name="pointColorThreshold">����ɫ��ֵ</param>
/// <param name="regionColorThreshold">������ɫ��ֵ</param>
/// <param name="minClusterSize">����С�ߴ�</param>
/// <param name="maxClusterSize">�����ߴ�</param>
/// <param name="threadsCount">�߳���</param>
/// <param name="clustersCount">���ƴ���</param>
/// <returns>���ƴ��б�</returns>
EXPORT_C Point3Color4s** CALLING_MODE regionGrowingColorSegment(Point3Color4 points[], int length, int normalK, int clusterK, float distanceThreshold, float smoothnessThreshold, float curvatureThreshold, float pointColorThreshold, float regionColorThreshold, int minClusterSize, int maxClusterSize, int threadsCount, int& clustersCount);
