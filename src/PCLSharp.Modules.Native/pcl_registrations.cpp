#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ia_fpcs.h>
#include <pcl/registration/ia_kfpcs.h>
#include <primitives_map.h>
#include "pcl_registrations.h"
using namespace std;
using namespace pcl;
using namespace pcl::registration;

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
AlignmentResult* align4PCS(Point3F sourcePoints[], const int sourceLength, Point3F targetPoints[], const int targetLength, const float approxOverlap, const float delta, const bool normalize, const int samplesCount, const int maxComputationTime, const int threadsCount)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(sourcePoints, sourceLength);
	const PointCloud<PointXYZ>::Ptr& targetCloud = pclsharp::toPointCloud(targetPoints, targetLength);

	//4PCS��׼
	PointCloud<PointXYZ> finalCloud;
	FPCSInitialAlignment<PointXYZ, PointXYZ> alignment;
	alignment.setInputSource(sourceCloud);
	alignment.setInputTarget(targetCloud);
	alignment.setApproxOverlap(approxOverlap);
	alignment.setDelta(delta, normalize);
	alignment.setNumberOfSamples(samplesCount);
	alignment.setMaxComputationTime(maxComputationTime);
	alignment.setNumberOfThreads(threadsCount);
	alignment.align(finalCloud);

	//������׼���
	const bool& hasConverged = alignment.hasConverged();
	const float& fitnessScore = alignment.getFitnessScore();
	const Eigen::Matrix4f& rtMatrix = alignment.getFinalTransformation();
	AlignmentResult* alignmentResult = new AlignmentResult(hasConverged, fitnessScore);

	const int& rowsCount = rtMatrix.rows();
	const int& colsCount = rtMatrix.cols();
	for (int rowIndex = 0; rowIndex < rowsCount; rowIndex++)
	{
		for (int colIndex = 0; colIndex < colsCount; colIndex++)
		{
			const int& index = rowIndex * colsCount + colIndex;
			const float& value = rtMatrix(rowIndex, colIndex);
			alignmentResult->Matrix[index] = value;
		}
	}

	return alignmentResult;
}

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
AlignmentResult* alignK4PCS(Point3F sourcePoints[], const int sourceLength, Point3F targetPoints[], const int targetLength, const float approxOverlap, const float lambda, const float delta, const bool normalize, const int samplesCount, const int maxComputationTime, const int threadsCount)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(sourcePoints, sourceLength);
	const PointCloud<PointXYZ>::Ptr& targetCloud = pclsharp::toPointCloud(targetPoints, targetLength);

	//K-4PCS��׼
	PointCloud<PointXYZ> finalCloud;
	KFPCSInitialAlignment<PointXYZ, PointXYZ> alignment;
	alignment.setInputSource(sourceCloud);
	alignment.setInputTarget(targetCloud);
	alignment.setApproxOverlap(approxOverlap);
	alignment.setLambda(lambda);
	alignment.setDelta(delta, normalize);
	alignment.setNumberOfSamples(samplesCount);
	alignment.setMaxComputationTime(maxComputationTime);
	alignment.setNumberOfThreads(threadsCount);
	alignment.align(finalCloud);

	//������׼���
	const bool& hasConverged = alignment.hasConverged();
	const float& fitnessScore = alignment.getFitnessScore();
	const Eigen::Matrix4f& rtMatrix = alignment.getFinalTransformation();
	AlignmentResult* alignmentResult = new AlignmentResult(hasConverged, fitnessScore);

	const int& rowsCount = rtMatrix.rows();
	const int& colsCount = rtMatrix.cols();
	for (int rowIndex = 0; rowIndex < rowsCount; rowIndex++)
	{
		for (int colIndex = 0; colIndex < colsCount; colIndex++)
		{
			const int& index = rowIndex * colsCount + colIndex;
			const float& value = rtMatrix(rowIndex, colIndex);
			alignmentResult->Matrix[index] = value;
		}
	}

	return alignmentResult;
}

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
AlignmentResult* alignNDT(Point3F sourcePoints[], int sourceLength, Point3F targetPoints[], int targetLength, float resolution, float stepSize, float transformationEpsilon, int maximumIterations)
{

}

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
AlignmentResult* CALLING_MODE alignICP(Point3F sourcePoints[], int sourceLength, Point3F targetPoints[], int targetLength, float maxCorrespondenceDistance, float transformationEpsilon, float euclideanFitnessEpsilon, int maximumIterations)
{

}

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
AlignmentResult* CALLING_MODE alignGICP(Point3F sourcePoints[], int sourceLength, Point3F targetPoints[], int targetLength, float maxCorrespondenceDistance, float transformationEpsilon, float euclideanFitnessEpsilon, int maximumIterations)
{

}

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
AlignmentResult* SaciaAlignNARF(Point3F sourcePoints[], Narf36F sourceDescriptors[], int sourceLength, Point3F targetPoints[], Narf36F targetDescriptors[], int targetLength, float minSampleDistance, int samplesCount, int correspondenceRandomness)
{

}

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
AlignmentResult* SaciaAlignPFH(Point3F sourcePoints[], PFHSignature125F sourceDescriptors[], int sourceLength, Point3F targetPoints[], PFHSignature125F targetDescriptors[], int targetLength, float minSampleDistance, int samplesCount, int correspondenceRandomness)
{

}

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
AlignmentResult* CALLING_MODE SaciaAlignFPFH(Point3F sourcePoints[], FPFHSignature33F sourceDescriptors[], int sourceLength, Point3F targetPoints[], FPFHSignature33F targetDescriptors[], int targetLength, float minSampleDistance, int samplesCount, int correspondenceRandomness)
{

}

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
AlignmentResult* SaciaAlign3DSC(Point3F sourcePoints[], ShapeContext1980F sourceDescriptors[], int sourceLength, Point3F targetPoints[], ShapeContext1980F targetDescriptors[], int targetLength, float minSampleDistance, int samplesCount, int correspondenceRandomness)
{

}

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
AlignmentResult* SaciaAlignSHOT(Point3F sourcePoints[], Shot352F sourceDescriptors[], int sourceLength, Point3F targetPoints[], Shot352F targetDescriptors[], int targetLength, float minSampleDistance, int samplesCount, int correspondenceRandomness)
{

}
