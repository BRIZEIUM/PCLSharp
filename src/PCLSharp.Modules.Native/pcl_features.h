#pragma once
#ifdef _WIN32
#define EXPORT_C extern "C" __declspec(dllexport)
#elif __linux__
#define EXPORT_C extern "C"
#endif
#define CALLING_MODE _cdecl
#include <point3f.h>
#include <narf36fs.h>
#include <pfh_signature125fs.h>
#include <fpfh_signature33fs.h>
#include <shape_context1980fs.h>
#include <shot352fs.h>

/// <summary>
/// ����NARF����
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
/// <param name="rotationInvariant">��ת������</param>
/// <returns>NARF���������Ӽ�</returns>
EXPORT_C Narf36Fs* CALLING_MODE computeNARF(Point3F points[], int length, float angularResolution, float maxAngleWidth, float maxAngleHeight, float noiseLevel, float minRange, int borderSize, float supportSize, bool rotationInvariant);

/// <summary>
/// ����PFH����
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="normalK">������K</param>
/// <param name="featureK">����K</param>
/// <param name="threadsCount">�߳���</param>
/// <returns>PFH���������Ӽ�</returns>
/// <remarks>����KֵҪ���ڷ�����Kֵ</remarks>
EXPORT_C PFHSignature125Fs* CALLING_MODE computePFH(Point3F points[], int length, int normalK, int featureK, int threadsCount);

/// <summary>
/// ����FPFH����
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="normalK">������K</param>
/// <param name="featureK">����K</param>
/// <param name="threadsCount">�߳���</param>
/// <returns>FPFH���������Ӽ�</returns>
/// <remarks>����KֵҪ���ڷ�����Kֵ</remarks>
EXPORT_C FPFHSignature33Fs* CALLING_MODE computeFPFH(Point3F points[], int length, int normalK, int featureK, int threadsCount);

/// <summary>
/// ����3DSC����
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="normalK">������K</param>
/// <param name="searchRadius">�����뾶</param>
/// <param name="pointDensityRadius">���ܶȰ뾶</param>
/// <param name="minimalRadius">��С�뾶</param>
/// <param name="threadsCount">�߳���</param>
/// <returns>3DSC���������Ӽ�</returns>
/// <remarks>
/// ���ܶȰ뾶��������Ϊ�����뾶��1/5��
///	��С�뾶��������Ϊ�����뾶��1/10��
/// </remarks>
EXPORT_C ShapeContext1980Fs* CALLING_MODE compute3DSC(Point3F points[], int length, int normalK, float searchRadius, float pointDensityRadius, float minimalRadius, int threadsCount);

/// <summary>
/// ����SHOT����
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="normalK">������K</param>
/// <param name="featureRadius">���������뾶</param>
/// <param name="threadsCount">�߳���</param>
/// <returns>SHOT���������Ӽ�</returns>
EXPORT_C Shot352Fs* CALLING_MODE computeSHOT(Point3F points[], int length, int normalK, float featureRadius, int threadsCount);
