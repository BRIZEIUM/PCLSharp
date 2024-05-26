#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/range_image/range_image.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3D.h>
#include <pcl/keypoints/susan.h>
#include <pcl/features/range_image_border_extractor.h>
#include <primitives_map.h>
#include "pcl_keypoints.h"
using namespace std;
using namespace pcl;

/* SIFT�ؼ������������� */
namespace pcl
{
	template<>
	struct SIFTKeypointFieldSelector<PointXYZ>
	{
		inline float operator()(const PointXYZ& point) const
		{
			return point.z;
		}
	};
}

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
Point3Fs* detectNARF(Point3F points[], const int length, const float angularResolution, const float maxAngleWidth, const float maxAngleHeight, const float noiseLevel, const float minRange, const int borderSize, const float supportSize)
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

	//��ȡNARF�ؼ���
	const size_t& keyPointsCount = keyPointsIndices.points.size();
	keyPoints->points.resize(keyPointsCount);
	for (int index = 0; index < keyPointsCount; index++)
	{
		keyPoints->points[index].getVector3fMap() = rangeImage->points[keyPointsIndices.points[index]].getVector3fMap();
	}

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*keyPoints);

	return point3Fs;
}

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
Point3Fs* detectISS(Point3F points[], const int length, const float salientRadius, const float nonMaxRadius, const float threshold21, const float threshold32, const int minNeighborsCount, const int threadsCount)
{
	const PointCloud<PointXYZ>::Ptr& cloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr keyPoints = std::make_shared<PointCloud<PointXYZ>>();

	//��ȡISS�ؼ���
	const search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();
	ISSKeypoint3D<PointXYZ, PointXYZ> issDetector;
	issDetector.setInputCloud(cloud);
	issDetector.setSearchMethod(kdTree);
	issDetector.setSalientRadius(salientRadius);
	issDetector.setNonMaxRadius(nonMaxRadius);
	issDetector.setThreshold21(threshold21);
	issDetector.setThreshold32(threshold32);
	issDetector.setMinNeighbors(minNeighborsCount);
	issDetector.setNumberOfThreads(threadsCount);
	issDetector.compute(*keyPoints);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*keyPoints);

	return point3Fs;
}

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
Point3Fs* detectSIFT(Point3F points[], const int length, const float minScale, const int octavesCount, const int scalesPerOctaveCount, const float minContrast)
{
	const PointCloud<PointXYZ>::Ptr cloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr keyPoints = std::make_shared<PointCloud<PointXYZ>>();

	//��ȡSIFT�ؼ���
	search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();
	PointCloud<PointWithScale> result;
	SIFTKeypoint<PointXYZ, PointWithScale> siftDetector;
	siftDetector.setInputCloud(cloud);
	siftDetector.setSearchMethod(kdTree);
	siftDetector.setScales(minScale, octavesCount, scalesPerOctaveCount);
	siftDetector.setMinimumContrast(minContrast);
	siftDetector.compute(result);
	pcl::copyPointCloud(result, *keyPoints);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*keyPoints);

	return point3Fs;
}

/// <summary>
/// ���Harris�ؼ���
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="nonMaxSupression">�Ǽ���ֵ����</param>
/// <param name="radius">�����뾶</param>
/// <param name="threshold">����Ȥ��ֵ</param>
/// <returns>Harris�ؼ��㼯</returns>
Point3Fs* detectHarris(Point3F points[], const int length, const bool nonMaxSupression, const float radius, const float threshold)
{
	const PointCloud<PointXYZ>::Ptr cloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr keyPoints = std::make_shared<PointCloud<PointXYZ>>();

	//��ȡHarris�ؼ���
	PointCloud<PointXYZI> result;
	HarrisKeypoint3D<PointXYZ, PointXYZI> harrisDetector;
	harrisDetector.setInputCloud(cloud);
	harrisDetector.setNonMaxSupression(nonMaxSupression);
	harrisDetector.setRadius(radius);
	harrisDetector.setThreshold(threshold);
	harrisDetector.compute(result);
	pcl::copyPointCloud(result, *keyPoints);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*keyPoints);

	return point3Fs;
}

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
Point3Fs* detectSUSAN(Point3F points[], const int length, const bool nonMaxSupression, const float radius, const float distanceThreshold, const float angularThreshold, const float intensityThreshold)
{
	const PointCloud<PointXYZ>::Ptr cloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr keyPoints = std::make_shared<PointCloud<PointXYZ>>();

	//��ȡSUSAN�ؼ���
	const search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();
	SUSANKeypoint<PointXYZ, PointXYZ> susanDetector;
	susanDetector.setInputCloud(cloud);
	susanDetector.setSearchMethod(kdTree);
	susanDetector.setNonMaxSupression(nonMaxSupression);
	susanDetector.setRadius(radius);
	susanDetector.setDistanceThreshold(distanceThreshold);
	susanDetector.setAngularThreshold(angularThreshold);
	susanDetector.setIntensityThreshold(intensityThreshold);
	susanDetector.compute(*keyPoints);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*keyPoints);

	return point3Fs;
}
