#pragma once
#define EXPORT_C extern "C" __declspec(dllexport)
#define CALLING_MODE _cdecl
#include <point3f.h>
#include <point3fs.h>

/// <summary>
/// ����ֱͨ�˲�
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="axis">����������</param>
/// <param name="limitMin">���˷�Χ��Сֵ</param>
/// <param name="limixMax">���˷�Χ���ֵ</param>
/// <returns>���˺�㼯</returns>
EXPORT_C Point3Fs* CALLING_MODE applyPassThrogh(Point3F points[], int length, const char* axis, float limitMin, float limixMax);

/// <summary>
/// �����������
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="seed">�������</param>
/// <param name="samplesCount">��������</param>
/// <returns>���˺�㼯</returns>
EXPORT_C Point3Fs* CALLING_MODE applyRandomSampling(Point3F points[], int length, int seed, int samplesCount);

/// <summary>
/// ���þ��Ȳ���
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="radius">�����뾶</param>
/// <returns>���˺�㼯</returns>
EXPORT_C Point3Fs* CALLING_MODE applyUniformSampling(Point3F points[], int length, float radius);

/// <summary>
/// �������ؽ�����
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="leafSize">Ҷ�ߴ�</param>
/// <returns>���˺�㼯</returns>
EXPORT_C Point3Fs* CALLING_MODE applyVoxelGrid(Point3F points[], int length, float leafSize);

/// <summary>
/// ���ý������ؽ�����
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="leafSize">Ҷ�ߴ�</param>
/// <returns>���˺�㼯</returns>
EXPORT_C Point3Fs* CALLING_MODE applyApproximateVoxelGrid(Point3F points[], int length, float leafSize);

/// <summary>
/// ����ͳ����Ⱥ���Ƴ�
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="meanK">ƽ��������Ƶ�����ھӵ�����</param>
/// <param name="stddevMult">��׼����ֵϵ��</param>
/// <returns>���˺�㼯</returns>
EXPORT_C Point3Fs* CALLING_MODE applyStatisticalOutlierRemoval(Point3F points[], int length, int meanK, float stddevMult);

/// <summary>
/// ���ð뾶��Ⱥ���Ƴ�
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="radius">�����뾶</param>
/// <param name="minNeighborsInRadius">�뾶��Χ�ڵ�������Сֵ</param>
/// <returns>���˺�㼯</returns>
EXPORT_C Point3Fs* CALLING_MODE applyRadiusOutlierRemoval(Point3F points[], int length, float radius, int minNeighborsInRadius);

/// <summary>
/// �ͷ���Դ
/// </summary>
/// <param name="pointer">ָ��</param>
EXPORT_C void CALLING_MODE dispose(const int* pointer);
