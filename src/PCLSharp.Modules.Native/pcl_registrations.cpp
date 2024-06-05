#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/ia_fpcs.h>
#include <pcl/registration/ia_kfpcs.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <primitives_map.h>
#include <features_map.h>
#include "pcl_registrations.h"
using namespace std;
using namespace pcl;
using namespace pcl::registration;

/// <summary>
/// FPCS��׼
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
AlignmentResult* alignFPCS(Point3F sourcePoints[], const int sourceLength, Point3F targetPoints[], const int targetLength, const float approxOverlap, const float delta, const bool normalize, const int samplesCount, const int maxComputationTime, const int threadsCount)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(sourcePoints, sourceLength);
	const PointCloud<PointXYZ>::Ptr& targetCloud = pclsharp::toPointCloud(targetPoints, targetLength);

	//FPCS��׼
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
/// K-FPCS��׼
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
AlignmentResult* alignKFPCS(Point3F sourcePoints[], const int sourceLength, Point3F targetPoints[], const int targetLength, const float approxOverlap, const float lambda, const float delta, const bool normalize, const int samplesCount, const int maxComputationTime, const int threadsCount)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(sourcePoints, sourceLength);
	const PointCloud<PointXYZ>::Ptr& targetCloud = pclsharp::toPointCloud(targetPoints, targetLength);

	//K-FPCS��׼
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
/// SAC-IA-NARF��׼
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
AlignmentResult* saciaAlignNARF(Point3F sourcePoints[], Narf36F sourceDescriptors[], const int sourceLength, Point3F targetPoints[], Narf36F targetDescriptors[], const int targetLength, const float minSampleDistance, const int samplesCount, const int correspondenceRandomness)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(sourcePoints, sourceLength);
	const PointCloud<PointXYZ>::Ptr& targetCloud = pclsharp::toPointCloud(targetPoints, targetLength);
	const PointCloud<Narf36>::Ptr& sourceFeatures = pclsharp::toPointCloud(sourceDescriptors, sourceLength);
	const PointCloud<Narf36>::Ptr& targetFeatures = pclsharp::toPointCloud(targetDescriptors, targetLength);

	//SAC-IA&NARF��׼
	PointCloud<PointXYZ> finalCloud;
	SampleConsensusInitialAlignment<PointXYZ, PointXYZ, Narf36> alignment;
	alignment.setInputSource(sourceCloud);
	alignment.setInputTarget(targetCloud);
	alignment.setSourceFeatures(sourceFeatures);
	alignment.setTargetFeatures(targetFeatures);
	alignment.setMinSampleDistance(minSampleDistance);
	alignment.setNumberOfSamples(samplesCount);
	alignment.setCorrespondenceRandomness(correspondenceRandomness);
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
/// SAC-IA-PFH��׼
/// </summary>
/// <param name="sourcePoints">Դ�㼯</param>
/// <param name="sourceDescriptors">ԴPFH�����Ӽ�</param>
/// <param name="sourceLength">Դ�㼯����</param>
/// <param name="targetPoints">Ŀ��㼯</param>
/// <param name="targetDescriptors">Ŀ��PFH�����Ӽ�</param>
/// <param name="targetLength">Ŀ��㼯����</param>
/// <param name="minSampleDistance">��������С����</param>
/// <param name="samplesCount">��������</param>
/// <param name="correspondenceRandomness">��������������</param>
/// <returns>��׼���</returns>
AlignmentResult* saciaAlignPFH(Point3F sourcePoints[], PFHSignature125F sourceDescriptors[], const int sourceLength, Point3F targetPoints[], PFHSignature125F targetDescriptors[], const int targetLength, const float minSampleDistance, const int samplesCount, const int correspondenceRandomness)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(sourcePoints, sourceLength);
	const PointCloud<PointXYZ>::Ptr& targetCloud = pclsharp::toPointCloud(targetPoints, targetLength);
	const PointCloud<PFHSignature125>::Ptr& sourceFeatures = pclsharp::toPointCloud(sourceDescriptors, sourceLength);
	const PointCloud<PFHSignature125>::Ptr& targetFeatures = pclsharp::toPointCloud(targetDescriptors, targetLength);

	//SAC-IA&PFH��׼
	PointCloud<PointXYZ> finalCloud;
	SampleConsensusInitialAlignment<PointXYZ, PointXYZ, PFHSignature125> alignment;
	alignment.setInputSource(sourceCloud);
	alignment.setInputTarget(targetCloud);
	alignment.setSourceFeatures(sourceFeatures);
	alignment.setTargetFeatures(targetFeatures);
	alignment.setMinSampleDistance(minSampleDistance);
	alignment.setNumberOfSamples(samplesCount);
	alignment.setCorrespondenceRandomness(correspondenceRandomness);
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
/// SAC-IA-FPFH��׼
/// </summary>
/// <param name="sourcePoints">Դ�㼯</param>
/// <param name="sourceDescriptors">ԴFPFH�����Ӽ�</param>
/// <param name="sourceLength">Դ�㼯����</param>
/// <param name="targetPoints">Ŀ��㼯</param>
/// <param name="targetDescriptors">Ŀ��FPFH�����Ӽ�</param>
/// <param name="targetLength">Ŀ��㼯����</param>
/// <param name="minSampleDistance">��������С����</param>
/// <param name="samplesCount">��������</param>
/// <param name="correspondenceRandomness">��������������</param>
/// <returns>��׼���</returns>
AlignmentResult* saciaAlignFPFH(Point3F sourcePoints[], FPFHSignature33F sourceDescriptors[], const int sourceLength, Point3F targetPoints[], FPFHSignature33F targetDescriptors[], const int targetLength, const float minSampleDistance, const int samplesCount, const int correspondenceRandomness)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(sourcePoints, sourceLength);
	const PointCloud<PointXYZ>::Ptr& targetCloud = pclsharp::toPointCloud(targetPoints, targetLength);
	const PointCloud<FPFHSignature33>::Ptr& sourceFeatures = pclsharp::toPointCloud(sourceDescriptors, sourceLength);
	const PointCloud<FPFHSignature33>::Ptr& targetFeatures = pclsharp::toPointCloud(targetDescriptors, targetLength);

	//SAC-IA&FPFH��׼
	PointCloud<PointXYZ> finalCloud;
	SampleConsensusInitialAlignment<PointXYZ, PointXYZ, FPFHSignature33> alignment;
	alignment.setInputSource(sourceCloud);
	alignment.setInputTarget(targetCloud);
	alignment.setSourceFeatures(sourceFeatures);
	alignment.setTargetFeatures(targetFeatures);
	alignment.setMinSampleDistance(minSampleDistance);
	alignment.setNumberOfSamples(samplesCount);
	alignment.setCorrespondenceRandomness(correspondenceRandomness);
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
/// SAC-IA-3DSC��׼
/// </summary>
/// <param name="sourcePoints">Դ�㼯</param>
/// <param name="sourceDescriptors">Դ3DSC�����Ӽ�</param>
/// <param name="sourceLength">Դ�㼯����</param>
/// <param name="targetPoints">Ŀ��㼯</param>
/// <param name="targetDescriptors">Ŀ��3DSC�����Ӽ�</param>
/// <param name="targetLength">Ŀ��㼯����</param>
/// <param name="minSampleDistance">��������С����</param>
/// <param name="samplesCount">��������</param>
/// <param name="correspondenceRandomness">��������������</param>
/// <returns>��׼���</returns>
AlignmentResult* saciaAlign3DSC(Point3F sourcePoints[], ShapeContext1980F sourceDescriptors[], const int sourceLength, Point3F targetPoints[], ShapeContext1980F targetDescriptors[], const int targetLength, const float minSampleDistance, const int samplesCount, const int correspondenceRandomness)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(sourcePoints, sourceLength);
	const PointCloud<PointXYZ>::Ptr& targetCloud = pclsharp::toPointCloud(targetPoints, targetLength);
	const PointCloud<ShapeContext1980>::Ptr& sourceFeatures = pclsharp::toPointCloud(sourceDescriptors, sourceLength);
	const PointCloud<ShapeContext1980>::Ptr& targetFeatures = pclsharp::toPointCloud(targetDescriptors, targetLength);

	//SAC-IA&3DSC��׼
	PointCloud<PointXYZ> finalCloud;
	SampleConsensusInitialAlignment<PointXYZ, PointXYZ, ShapeContext1980> alignment;
	alignment.setInputSource(sourceCloud);
	alignment.setInputTarget(targetCloud);
	alignment.setSourceFeatures(sourceFeatures);
	alignment.setTargetFeatures(targetFeatures);
	alignment.setMinSampleDistance(minSampleDistance);
	alignment.setNumberOfSamples(samplesCount);
	alignment.setCorrespondenceRandomness(correspondenceRandomness);
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
/// SAC-IA-SHOT��׼
/// </summary>
/// <param name="sourcePoints">Դ�㼯</param>
/// <param name="sourceDescriptors">ԴSHOT�����Ӽ�</param>
/// <param name="sourceLength">Դ�㼯����</param>
/// <param name="targetPoints">Ŀ��㼯</param>
/// <param name="targetDescriptors">Ŀ��SHOT�����Ӽ�</param>
/// <param name="targetLength">Ŀ��㼯����</param>
/// <param name="minSampleDistance">��������С����</param>
/// <param name="samplesCount">��������</param>
/// <param name="correspondenceRandomness">��������������</param>
/// <returns>��׼���</returns>
AlignmentResult* saciaAlignSHOT(Point3F sourcePoints[], Shot352F sourceDescriptors[], const int sourceLength, Point3F targetPoints[], Shot352F targetDescriptors[], const int targetLength, const float minSampleDistance, const int samplesCount, const int correspondenceRandomness)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(sourcePoints, sourceLength);
	const PointCloud<PointXYZ>::Ptr& targetCloud = pclsharp::toPointCloud(targetPoints, targetLength);
	const PointCloud<SHOT352>::Ptr& sourceFeatures = pclsharp::toPointCloud(sourceDescriptors, sourceLength);
	const PointCloud<SHOT352>::Ptr& targetFeatures = pclsharp::toPointCloud(targetDescriptors, targetLength);

	//SAC-IA&SHOT��׼
	PointCloud<PointXYZ> finalCloud;
	SampleConsensusInitialAlignment<PointXYZ, PointXYZ, SHOT352> alignment;
	alignment.setInputSource(sourceCloud);
	alignment.setInputTarget(targetCloud);
	alignment.setSourceFeatures(sourceFeatures);
	alignment.setTargetFeatures(targetFeatures);
	alignment.setMinSampleDistance(minSampleDistance);
	alignment.setNumberOfSamples(samplesCount);
	alignment.setCorrespondenceRandomness(correspondenceRandomness);
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
AlignmentResult* alignNDT(Point3F sourcePoints[], const int sourceLength, Point3F targetPoints[], const int targetLength, const float resolution, const float stepSize, const float transformationEpsilon, const int maximumIterations)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(sourcePoints, sourceLength);
	const PointCloud<PointXYZ>::Ptr& targetCloud = pclsharp::toPointCloud(targetPoints, targetLength);

	//NDT��׼
	PointCloud<PointXYZ> finalCloud;
	NormalDistributionsTransform<PointXYZ, PointXYZ> alignment;
	alignment.setInputSource(sourceCloud);
	alignment.setInputTarget(targetCloud);
	alignment.setResolution(resolution);
	alignment.setStepSize(stepSize);
	alignment.setTransformationEpsilon(transformationEpsilon);
	alignment.setMaximumIterations(maximumIterations);
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
/// GICP��׼
/// </summary>
/// <param name="sourcePoints">Դ�㼯</param>
/// <param name="sourceLength">Դ�㼯����</param>
/// <param name="targetPoints">Ŀ��㼯</param>
/// <param name="targetLength">Ŀ��㼯����</param>
/// <param name="maxCorrespondenceDistance">������ƾ���</param>
/// <param name="transformationEpsilon">�任����ֵ</param>
/// <param name="euclideanFitnessEpsilon">���������ֵ</param>
/// <param name="maximumIterations">����������</param>
/// <returns>��׼���</returns>
AlignmentResult* alignGICP(Point3F sourcePoints[], const int sourceLength, Point3F targetPoints[], const int targetLength, const float maxCorrespondenceDistance, const float transformationEpsilon, const float euclideanFitnessEpsilon, const int maximumIterations)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(sourcePoints, sourceLength);
	const PointCloud<PointXYZ>::Ptr& targetCloud = pclsharp::toPointCloud(targetPoints, targetLength);

	//GICP��׼
	PointCloud<PointXYZ> finalCloud;
	GeneralizedIterativeClosestPoint<PointXYZ, PointXYZ> alignment;
	alignment.setInputSource(sourceCloud);
	alignment.setInputTarget(targetCloud);
	alignment.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
	alignment.setTransformationEpsilon(transformationEpsilon);
	alignment.setEuclideanFitnessEpsilon(euclideanFitnessEpsilon);
	alignment.setMaximumIterations(maximumIterations);
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
/// ICP-Point-To-Point��׼
/// </summary>
/// <param name="sourcePoints">Դ�㼯</param>
/// <param name="sourceLength">Դ�㼯����</param>
/// <param name="targetPoints">Ŀ��㼯</param>
/// <param name="targetLength">Ŀ��㼯����</param>
/// <param name="maxCorrespondenceDistance">������ƾ���</param>
/// <param name="transformationEpsilon">�任����ֵ</param>
/// <param name="euclideanFitnessEpsilon">���������ֵ</param>
/// <param name="maximumIterations">����������</param>
/// <returns>��׼���</returns>
AlignmentResult* alignPointToPoint(Point3F sourcePoints[], const int sourceLength, Point3F targetPoints[], const int targetLength, const float maxCorrespondenceDistance, const float transformationEpsilon, const float euclideanFitnessEpsilon, const int maximumIterations)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(sourcePoints, sourceLength);
	const PointCloud<PointXYZ>::Ptr& targetCloud = pclsharp::toPointCloud(targetPoints, targetLength);

	//ICP��׼
	PointCloud<PointXYZ> finalCloud;
	IterativeClosestPoint<PointXYZ, PointXYZ> alignment;
	alignment.setInputSource(sourceCloud);
	alignment.setInputTarget(targetCloud);
	alignment.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
	alignment.setTransformationEpsilon(transformationEpsilon);
	alignment.setEuclideanFitnessEpsilon(euclideanFitnessEpsilon);
	alignment.setMaximumIterations(maximumIterations);
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
/// ICP-Point-To-Plane��׼
/// </summary>
/// <param name="sourcePoints">Դ�㼯</param>
/// <param name="sourceLength">Դ�㼯����</param>
/// <param name="targetPoints">Ŀ��㼯</param>
/// <param name="targetLength">Ŀ��㼯����</param>
/// <param name="normalK">������K</param>
/// <param name="maxCorrespondenceDistance">������ƾ���</param>
/// <param name="transformationEpsilon">�任����ֵ</param>
/// <param name="euclideanFitnessEpsilon">���������ֵ</param>
/// <param name="maximumIterations">����������</param>
/// <param name="threadsCount">�߳���</param>
/// <returns>��׼���</returns>
AlignmentResult* alignPointToPlane(Point3F sourcePoints[], const int sourceLength, Point3F targetPoints[], const int targetLength, const int normalK, const float maxCorrespondenceDistance, const float transformationEpsilon, const float euclideanFitnessEpsilon, const int maximumIterations, const int threadsCount)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(sourcePoints, sourceLength);
	const PointCloud<PointXYZ>::Ptr& targetCloud = pclsharp::toPointCloud(targetPoints, targetLength);
	const PointCloud<Normal>::Ptr sourceNormals = std::make_shared<PointCloud<Normal>>();
	const PointCloud<Normal>::Ptr targetNormals = std::make_shared<PointCloud<Normal>>();
	const PointCloud<PointNormal>::Ptr sourcePointNormals = std::make_shared<PointCloud<PointNormal>>();
	const PointCloud<PointNormal>::Ptr targetPointNormals = std::make_shared<PointCloud<PointNormal>>();
	const search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();

	//���㷨����
	NormalEstimationOMP<PointXYZ, Normal> normalEstimator;
	normalEstimator.setSearchMethod(kdTree);
	normalEstimator.setKSearch(normalK);
	normalEstimator.setNumberOfThreads(threadsCount);
	normalEstimator.setInputCloud(sourceCloud);
	normalEstimator.compute(*sourceNormals);
	normalEstimator.setInputCloud(targetCloud);
	normalEstimator.compute(*targetNormals);
	pcl::concatenateFields(*sourceCloud, *sourceNormals, *sourcePointNormals);
	pcl::concatenateFields(*targetCloud, *targetNormals, *targetPointNormals);

	//ICP��׼
	PointCloud<PointNormal> finalCloud;
	IterativeClosestPointWithNormals<PointNormal, PointNormal> alignment;
	alignment.setInputSource(sourcePointNormals);
	alignment.setInputTarget(targetPointNormals);
	alignment.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
	alignment.setTransformationEpsilon(transformationEpsilon);
	alignment.setEuclideanFitnessEpsilon(euclideanFitnessEpsilon);
	alignment.setMaximumIterations(maximumIterations);
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
