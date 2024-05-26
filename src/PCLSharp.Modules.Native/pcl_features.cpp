#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/shot_omp.h>
#include <primitives_map.h>
#include <features_map.h>
#include "pcl_features.h"
using namespace std;
using namespace pcl;

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
/// <remarks>����KֵҪ���ڷ�����Kֵ</remarks>
Narf36Fs* computeNARF(Point3F points[], const int length, const float angularResolution, const float maxAngleWidth, const float maxAngleHeight, const float noiseLevel, const float minRange, const int borderSize, const float supportSize, const bool rotationInvariant)
{
	const PointCloud<PointXYZ>::Ptr cloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr keyPoints = std::make_shared<PointCloud<PointXYZ>>();
	const RangeImage::Ptr rangeImage = std::make_shared<RangeImage>();

	//�������ͼ
	const Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Affine3f::Identity());
	rangeImage->createFromPointCloud(*cloud, deg2rad(angularResolution), deg2rad(maxAngleWidth), deg2rad(maxAngleHeight), sensorPose, RangeImage::CAMERA_FRAME, noiseLevel, minRange, borderSize);

	//��ȡNARF�ؼ�������
	PointCloud<int> keyPointsIndices;
	RangeImageBorderExtractor rangeImageBorderExtractor;
	NarfKeypoint narfDetector = NarfKeypoint(&rangeImageBorderExtractor);
	narfDetector.setRangeImage(&*rangeImage);
	NarfKeypoint::Parameters& detectorParameters = narfDetector.getParameters();
	detectorParameters.support_size = supportSize;
	narfDetector.compute(keyPointsIndices);

	//����NARF�ؼ�������
	vector<int> keyPointsIndicesV;
	keyPointsIndicesV.resize(keyPointsIndices.points.size());
	for (int i = 0; i < keyPointsIndices.size(); i++)
	{
		keyPointsIndicesV[i] = keyPointsIndices.points[i];
	}

	//��ȡNARF������
	PointCloud<Narf36> descriptors;
	NarfDescriptor narfComputer = NarfDescriptor(&*rangeImage, &keyPointsIndicesV);
	NarfDescriptor::Parameters& computerParameters = narfComputer.getParameters();
	computerParameters.support_size = supportSize;
	computerParameters.rotation_invariant = rotationInvariant;
	narfComputer.compute(descriptors);

	Narf36Fs* narf36Fs = pclsharp::toNarf36Fs(descriptors);

	return narf36Fs;
}

/// <summary>
/// ����PFH����
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="normalK">������K</param>
/// <param name="featureK">����K</param>
/// <param name="threadsCount">�߳���</param>
/// <returns>PFH���������Ӽ�</returns>
PFHSignature125Fs* computePFH(Point3F points[], const int length, const int normalK, const int featureK, const int threadsCount)
{
	const PointCloud<PointXYZ>::Ptr cloud = pclsharp::toPointCloud(points, length);
	const PointCloud<Normal>::Ptr normals = std::make_shared<PointCloud<Normal>>();
	const PointCloud<PFHSignature125>::Ptr descriptors = std::make_shared<PointCloud<PFHSignature125>>();
	const search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();

	//���㷨����
	NormalEstimationOMP<PointXYZ, Normal> normalEstimator;
	normalEstimator.setInputCloud(cloud);
	normalEstimator.setSearchMethod(kdTree);
	normalEstimator.setKSearch(normalK);
	normalEstimator.setNumberOfThreads(threadsCount);
	normalEstimator.compute(*normals);

	//��ȡPFH������
	PFHEstimation<PointXYZ, Normal> pfhComputer;
	pfhComputer.setInputCloud(cloud);
	pfhComputer.setInputNormals(normals);
	pfhComputer.setSearchMethod(kdTree);
	pfhComputer.setKSearch(featureK);
	pfhComputer.compute(*descriptors);

	PFHSignature125Fs* signature125Fs = pclsharp::toPFHSignature125Fs(*descriptors);

	return signature125Fs;
}

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
FPFHSignature33Fs* computeFPFH(Point3F points[], const int length, const int normalK, const int featureK, const int threadsCount)
{
	const PointCloud<PointXYZ>::Ptr& cloud = pclsharp::toPointCloud(points, length);
	const PointCloud<Normal>::Ptr normals = std::make_shared<PointCloud<Normal>>();
	const PointCloud<FPFHSignature33>::Ptr descriptors = std::make_shared<PointCloud<FPFHSignature33>>();
	const search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();

	//���㷨����
	NormalEstimationOMP<PointXYZ, Normal> normalEstimator;
	normalEstimator.setInputCloud(cloud);
	normalEstimator.setSearchMethod(kdTree);
	normalEstimator.setKSearch(normalK);
	normalEstimator.setNumberOfThreads(threadsCount);
	normalEstimator.compute(*normals);

	//��ȡFPFH������
	FPFHEstimationOMP<PointXYZ, Normal, FPFHSignature33> fpfhComputer;
	fpfhComputer.setInputCloud(cloud);
	fpfhComputer.setInputNormals(normals);
	fpfhComputer.setSearchMethod(kdTree);
	fpfhComputer.setKSearch(featureK);
	fpfhComputer.setNumberOfThreads(threadsCount);
	fpfhComputer.compute(*descriptors);

	FPFHSignature33Fs* signature33Fs = pclsharp::toFPFHSignature33Fs(*descriptors);

	return signature33Fs;
}

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
ShapeContext1980Fs* compute3DSC(Point3F points[], const int length, const int normalK, const float searchRadius, const float pointDensityRadius, const float minimalRadius, const int threadsCount)
{
	const PointCloud<PointXYZ>::Ptr& cloud = pclsharp::toPointCloud(points, length);
	const PointCloud<Normal>::Ptr normals = std::make_shared<PointCloud<Normal>>();
	const PointCloud<ShapeContext1980>::Ptr descriptors = std::make_shared<PointCloud<ShapeContext1980>>();
	const search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();

	//���㷨����
	NormalEstimationOMP<PointXYZ, Normal> normalEstimator;
	normalEstimator.setInputCloud(cloud);
	normalEstimator.setSearchMethod(kdTree);
	normalEstimator.setKSearch(normalK);
	normalEstimator.setNumberOfThreads(threadsCount);
	normalEstimator.compute(*normals);

	//��ȡ3DSC����������
	ShapeContext3DEstimation<PointXYZ, Normal, ShapeContext1980> dsc3Computer;
	dsc3Computer.setInputCloud(cloud);
	dsc3Computer.setInputNormals(normals);
	dsc3Computer.setSearchMethod(kdTree);
	dsc3Computer.setRadiusSearch(searchRadius);
	dsc3Computer.setPointDensityRadius(pointDensityRadius);
	dsc3Computer.setMinimalRadius(minimalRadius);
	dsc3Computer.compute(*descriptors);

	ShapeContext1980Fs* shapeContext1980Fs = pclsharp::toShapeContext1980Fs(*descriptors);

	return shapeContext1980Fs;
}

/// <summary>
/// ����SHOT����
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="normalK">������K</param>
/// <param name="featureRadius">���������뾶</param>
/// <param name="threadsCount">�߳���</param>
/// <returns>SHOT���������Ӽ�</returns>
EXPORT_C Shot352Fs* CALLING_MODE computeSHOT(Point3F points[], const int length, const int normalK, const float featureRadius, const int threadsCount)
{
	const PointCloud<PointXYZ>::Ptr& cloud = pclsharp::toPointCloud(points, length);
	const PointCloud<Normal>::Ptr normals = std::make_shared<PointCloud<Normal>>();
	const PointCloud<SHOT352>::Ptr descriptors = std::make_shared<PointCloud<SHOT352>>();
	const search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();

	//���㷨����
	NormalEstimationOMP<PointXYZ, Normal> normalEstimator;
	normalEstimator.setInputCloud(cloud);
	normalEstimator.setSearchMethod(kdTree);
	normalEstimator.setKSearch(normalK);
	normalEstimator.setNumberOfThreads(threadsCount);
	normalEstimator.compute(*normals);

	//��ȡSHOT����������
	SHOTEstimationOMP<PointXYZ, Normal> shotComputer;
	shotComputer.setInputCloud(cloud);
	shotComputer.setInputNormals(normals);
	shotComputer.setRadiusSearch(featureRadius);
	shotComputer.setNumberOfThreads(threadsCount);
	shotComputer.compute(*descriptors);

	Shot352Fs* shot352Fs = pclsharp::toShot352Fs(*descriptors);

	return shot352Fs;
}
