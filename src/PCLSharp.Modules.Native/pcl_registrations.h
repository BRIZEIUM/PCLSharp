#pragma once
#define EXPORT_C extern "C" __declspec(dllexport)
#define CALLING_MODE _cdecl
#include <point3f.h>
#include <narf36f.h>
#include <pfh_signature125f.h>
#include <fpfh_signature33f.h>
#include <shape_context1980f.h>
#include <shot352f.h>
#include <alignment_result.h>

/// <summary>
/// 4PCS��׼
/// </summary>
/// <param name="sourcePoints">Դ�㼯</param>
/// <param name="sourceLength">Դ�㼯����</param>
/// <param name="targetPoints">Ŀ��㼯</param>
/// <param name="targetLength">Ŀ��㼯����</param>
/// <param name="approxOverlap">�����ص�</param>
/// <param name="delta">��׼����</param>
/// <param name="normalize">��׼��</param>
/// <param name="samplesCount">��������</param>
/// <param name="maxComputationTime">������ʱ��(��)</param>
/// <param name="threadsCount">�߳���</param>
/// <returns>��׼���</returns>
EXPORT_C AlignmentResult* CALLING_MODE align4PCS(Point3F sourcePoints[], int sourceLength, Point3F targetPoints[], int targetLength, float approxOverlap, float delta, bool normalize, int samplesCount, int maxComputationTime, int threadsCount);

/// <summary>
/// K-4PCS��׼
/// </summary>
/// <param name="sourcePoints">Դ�㼯</param>
/// <param name="sourceLength">Դ�㼯����</param>
/// <param name="targetPoints">Ŀ��㼯</param>
/// <param name="targetLength">Ŀ��㼯����</param>
/// <param name="approxOverlap">�����ص�</param>
/// <param name="lambda">ƽ��������Ȩϵ��</param>
/// <param name="delta">��׼����</param>
/// <param name="normalize">��׼��</param>
/// <param name="samplesCount">��������</param>
/// <param name="maxComputationTime">������ʱ��(��)</param>
/// <param name="threadsCount">�߳���</param>
/// <returns>��׼���</returns>
EXPORT_C AlignmentResult* CALLING_MODE alignK4PCS(Point3F sourcePoints[], int sourceLength, Point3F targetPoints[], int targetLength, float approxOverlap, float lambda, float delta, bool normalize, int samplesCount, int maxComputationTime, int threadsCount);

/// <summary>
/// NDT��׼
/// </summary>
/// <param name="sourcePoints">Դ�㼯</param>
/// <param name="sourceLength">Դ�㼯����</param>
/// <param name="targetPoints">Ŀ��㼯</param>
/// <param name="targetLength">Ŀ��㼯����</param>
/// <param name="resolution">�ֱ���</param>
/// <param name="stepSize">����</param>
/// <param name="transformationEpsilon">�任����ֵ</param>
/// <param name="maximumIterations">����������</param>
/// <returns>��׼���</returns>
EXPORT_C AlignmentResult* CALLING_MODE alignNDT(Point3F sourcePoints[], int sourceLength, Point3F targetPoints[], int targetLength, float resolution, float stepSize, float transformationEpsilon, int maximumIterations);

/// <summary>
/// ICP��׼
/// </summary>
/// <param name="sourcePoints">Դ�㼯</param>
/// <param name="sourceLength">Դ�㼯����</param>
/// <param name="targetPoints">Ŀ��㼯</param>
/// <param name="targetLength">Ŀ��㼯����</param>
/// <param name="maxCorrespondenceDistance">�ֱ���</param>
/// <param name="transformationEpsilon">�任����ֵ</param>
/// <param name="euclideanFitnessEpsilon">���������ֵ</param>
/// <param name="maximumIterations">����������</param>
/// <returns>��׼���</returns>
EXPORT_C AlignmentResult* CALLING_MODE alignICP(Point3F sourcePoints[], int sourceLength, Point3F targetPoints[], int targetLength, float maxCorrespondenceDistance, float transformationEpsilon, float euclideanFitnessEpsilon, int maximumIterations);

/// <summary>
/// GICP��׼
/// </summary>
/// <param name="sourcePoints">Դ�㼯</param>
/// <param name="sourceLength">Դ�㼯����</param>
/// <param name="targetPoints">Ŀ��㼯</param>
/// <param name="targetLength">Ŀ��㼯����</param>
/// <param name="maxCorrespondenceDistance">�ֱ���</param>
/// <param name="transformationEpsilon">�任����ֵ</param>
/// <param name="euclideanFitnessEpsilon">���������ֵ</param>
/// <param name="maximumIterations">����������</param>
/// <returns>��׼���</returns>
EXPORT_C AlignmentResult* CALLING_MODE alignGICP(Point3F sourcePoints[], int sourceLength, Point3F targetPoints[], int targetLength, float maxCorrespondenceDistance, float transformationEpsilon, float euclideanFitnessEpsilon, int maximumIterations);

/// <summary>
/// SAC-IA&NARF��׼
/// </summary>
/// <param name="sourcePoints">Դ�㼯</param>
/// <param name="sourceDescriptors">ԴNARF�����Ӽ�</param>
/// <param name="sourceLength">Դ�㼯����</param>
/// <param name="targetPoints">Ŀ��㼯</param>
/// <param name="targetDescriptors">Ŀ��NARF�����Ӽ�</param>
/// <param name="targetLength">Ŀ��㼯����</param>
/// <param name="minSampleDistance">��������С����</param>
/// <param name="samplesCount">��������</param>
/// <param name="correspondenceRandomness">��������������</param>
/// <returns>��׼���</returns>
EXPORT_C AlignmentResult* CALLING_MODE SaciaAlignNARF(Point3F sourcePoints[], Narf36F sourceDescriptors[], int sourceLength, Point3F targetPoints[], Narf36F targetDescriptors[], int targetLength, float minSampleDistance, int samplesCount, int correspondenceRandomness);

/// <summary>
/// SAC-IA&PFH��׼
/// </summary>
/// <param name="sourcePoints">Դ�㼯</param>
/// <param name="sourceDescriptors">ԴNARF�����Ӽ�</param>
/// <param name="sourceLength">Դ�㼯����</param>
/// <param name="targetPoints">Ŀ��㼯</param>
/// <param name="targetDescriptors">Ŀ��NARF�����Ӽ�</param>
/// <param name="targetLength">Ŀ��㼯����</param>
/// <param name="minSampleDistance">��������С����</param>
/// <param name="samplesCount">��������</param>
/// <param name="correspondenceRandomness">��������������</param>
/// <returns>��׼���</returns>
EXPORT_C AlignmentResult* CALLING_MODE SaciaAlignPFH(Point3F sourcePoints[], PFHSignature125F sourceDescriptors[], int sourceLength, Point3F targetPoints[], PFHSignature125F targetDescriptors[], int targetLength, float minSampleDistance, int samplesCount, int correspondenceRandomness);

/// <summary>
/// SAC-IA&FPFH��׼
/// </summary>
/// <param name="sourcePoints">Դ�㼯</param>
/// <param name="sourceDescriptors">ԴNARF�����Ӽ�</param>
/// <param name="sourceLength">Դ�㼯����</param>
/// <param name="targetPoints">Ŀ��㼯</param>
/// <param name="targetDescriptors">Ŀ��NARF�����Ӽ�</param>
/// <param name="targetLength">Ŀ��㼯����</param>
/// <param name="minSampleDistance">��������С����</param>
/// <param name="samplesCount">��������</param>
/// <param name="correspondenceRandomness">��������������</param>
/// <returns>��׼���</returns>
EXPORT_C AlignmentResult* CALLING_MODE SaciaAlignFPFH(Point3F sourcePoints[], FPFHSignature33F sourceDescriptors[], int sourceLength, Point3F targetPoints[], FPFHSignature33F targetDescriptors[], int targetLength, float minSampleDistance, int samplesCount, int correspondenceRandomness);

/// <summary>
/// SAC-IA&3DSC��׼
/// </summary>
/// <param name="sourcePoints">Դ�㼯</param>
/// <param name="sourceDescriptors">ԴNARF�����Ӽ�</param>
/// <param name="sourceLength">Դ�㼯����</param>
/// <param name="targetPoints">Ŀ��㼯</param>
/// <param name="targetDescriptors">Ŀ��NARF�����Ӽ�</param>
/// <param name="targetLength">Ŀ��㼯����</param>
/// <param name="minSampleDistance">��������С����</param>
/// <param name="samplesCount">��������</param>
/// <param name="correspondenceRandomness">��������������</param>
/// <returns>��׼���</returns>
EXPORT_C AlignmentResult* CALLING_MODE SaciaAlign3DSC(Point3F sourcePoints[], ShapeContext1980F sourceDescriptors[], int sourceLength, Point3F targetPoints[], ShapeContext1980F targetDescriptors[], int targetLength, float minSampleDistance, int samplesCount, int correspondenceRandomness);

/// <summary>
/// SAC-IA&SHOT��׼
/// </summary>
/// <param name="sourcePoints">Դ�㼯</param>
/// <param name="sourceDescriptors">ԴNARF�����Ӽ�</param>
/// <param name="sourceLength">Դ�㼯����</param>
/// <param name="targetPoints">Ŀ��㼯</param>
/// <param name="targetDescriptors">Ŀ��NARF�����Ӽ�</param>
/// <param name="targetLength">Ŀ��㼯����</param>
/// <param name="minSampleDistance">��������С����</param>
/// <param name="samplesCount">��������</param>
/// <param name="correspondenceRandomness">��������������</param>
/// <returns>��׼���</returns>
EXPORT_C AlignmentResult* CALLING_MODE SaciaAlignSHOT(Point3F sourcePoints[], Shot352F sourceDescriptors[], int sourceLength, Point3F targetPoints[], Shot352F targetDescriptors[], int targetLength, float minSampleDistance, int samplesCount, int correspondenceRandomness);
