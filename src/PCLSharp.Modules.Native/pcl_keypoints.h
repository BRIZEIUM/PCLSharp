#pragma once
#define EXPORT_C extern "C" __declspec(dllexport)
#define CALLING_MODE _cdecl
#include <point3f.h>
#include <point3fs.h>

/// <summary>
/// ���NARF�ؼ���
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="angularResolution">�Ƕȷֱ���</param>
/// <param name="maxAngleWidth">������ˮƽ�߽�Ƕ�</param>
/// <param name="maxAngleHeight">��������ֱ�߽�Ƕ�</param>
/// <param name="noiseLevel">������������������</param>
/// <param name="minRange">��С�ɼ���Χ</param>
/// <param name="borderSize">�߽�ߴ�</param>
/// <param name="supportSize">���㷶Χ�뾶</param>
/// <returns>NARF�ؼ��㼯</returns>
EXPORT_C Point3Fs* CALLING_MODE detectNARF(Point3F points[], int length, float angularResolution, float maxAngleWidth, float maxAngleHeight, float noiseLevel, float minRange, int borderSize, float supportSize);

/// <summary>
/// ���ISS�ؼ���
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="salientRadius">�����뾶</param>
/// <param name="nonMaxRadius">�Ǽ���ֵ���ư뾶</param>
/// <param name="threshold21">��һ����ֵ������</param>
/// <param name="threshold32">��������ֵ������</param>
/// <param name="minNeighborsCount">��С�������</param>
/// <param name="threadsCount">�߳���</param>
/// <returns>ISS�ؼ��㼯</returns>
EXPORT_C Point3Fs* CALLING_MODE detectISS(Point3F points[], int length, float salientRadius, float nonMaxRadius, float threshold21, float threshold32, int minNeighborsCount, int threadsCount);

/// <summary>
/// ���SIFT�ؼ���
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="minScale">�߶ȿռ���С��׼ƫ��</param>
/// <param name="octavesCount">������������</param>
/// <param name="scalesPerOctaveCount">ÿ�����������߶�</param>
/// <param name="minContrast">���ƹؼ�������ֵ</param>
/// <returns>SIFT�ؼ��㼯</returns>
EXPORT_C Point3Fs* CALLING_MODE detectSIFT(Point3F points[], int length, float minScale, int octavesCount, int scalesPerOctaveCount, float minContrast);

/// <summary>
/// ���Harris�ؼ���
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="nonMaxSupression">�Ǽ���ֵ����</param>
/// <param name="radius">�����뾶</param>
/// <param name="threshold">����Ȥ��ֵ</param>
/// <returns>Harris�ؼ��㼯</returns>
EXPORT_C Point3Fs* CALLING_MODE detectHarris(Point3F points[], int length, bool nonMaxSupression, float radius, float threshold);

/// <summary>
/// ���SUSAN�ؼ���
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="nonMaxSupression">�Ǽ���ֵ����</param>
/// <param name="radius">�����뾶</param>
/// <param name="distanceThreshold">������ֵ</param>
/// <param name="angularThreshold">�Ƕ���ֵ</param>
/// <param name="intensityThreshold">ǿ����ֵ</param>
/// <returns>SUSAN�ؼ��㼯</returns>
EXPORT_C Point3Fs* CALLING_MODE detectSUSAN(Point3F points[], int length, bool nonMaxSupression, float radius, float distanceThreshold, float angularThreshold, float intensityThreshold);
