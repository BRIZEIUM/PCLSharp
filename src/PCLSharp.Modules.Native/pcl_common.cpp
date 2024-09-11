#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/features/boundary.h>
#include <pcl/surface/convex_hull.h>
#include <primitives_map.h>
#include "pcl_common.h"
using namespace std;
using namespace pcl;

/// <summary>
/// ��������
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <returns>���������</returns>
Point3F* estimateCentroid(Point3F points[], const int length)
{
	//���ص���
	const PointCloud<PointXYZ>::Ptr& cloud = pclsharp::toPointCloud(points, length);

	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloud, centroid);

	Point3F* point3F = new Point3F(centroid[0], centroid[1], centroid[2]);

	return point3F;
}

/// <summary>
/// ����任
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="pose">λ��</param>
/// <returns>�任�����</returns>
Point3Fs* affineTransform(Point3F points[], const int length, const Pose pose)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//��������任����
	Eigen::Affine3f affine = Eigen::Affine3f::Identity();

	//ƽ��
	affine.translation() << pose.X, pose.Y, pose.Z;

	//��ת
	const Eigen::AngleAxisf& rotationX = Eigen::AngleAxisf(pose.RX, Eigen::Vector3f::UnitX());
	const Eigen::AngleAxisf& rotationY = Eigen::AngleAxisf(pose.RY, Eigen::Vector3f::UnitY());
	const Eigen::AngleAxisf& rotationZ = Eigen::AngleAxisf(pose.RZ, Eigen::Vector3f::UnitZ());
	affine.rotate(rotationX);
	affine.rotate(rotationY);
	affine.rotate(rotationZ);

	//ִ�б任
	pcl::transformPointCloud(*sourceCloud, *targetCloud, affine);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// ����任
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="matrixArray">��������(����: 16)</param>
/// <returns>�任�����</returns>
Point3Fs* matrixTransform(Point3F points[], const int length, float matrixArray[])
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	Eigen::Matrix4f rtMatrix = Eigen::Matrix4f::Identity();
	const int& rowsCount = rtMatrix.rows();
	const int& colsCount = rtMatrix.cols();
	for (int rowIndex = 0; rowIndex < rowsCount; rowIndex++)
	{
		for (int colIndex = 0; colIndex < colsCount; colIndex++)
		{
			const int& index = rowIndex * colsCount + colIndex;
			rtMatrix(rowIndex, colIndex) = matrixArray[index];
		}
	}

	//ִ�б任
	pcl::transformPointCloud(*sourceCloud, *targetCloud, rtMatrix);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// �ϲ�����㷨����
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="normal3Fs">��������</param>
/// <param name="length">�㼯����</param>
/// <returns>�㼯</returns>
Point3Normal3s* mergePointsNormals(Point3F points[], Normal3F normal3Fs[], const int length)
{
	const PointCloud<PointXYZ>::Ptr& cloud = pclsharp::toPointCloud(points, length);
	const PointCloud<Normal>::Ptr& normals = pclsharp::toPointCloud(normal3Fs, length);
	PointCloud<PointNormal> pointNormals;
	pcl::concatenateFields(*cloud, *normals, pointNormals);

	Point3Normal3s* point3Normal3s = pclsharp::toPoint3Normal3s(pointNormals);

	return point3Normal3s;
}

/// <summary>
/// ���������
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="minPoint">��С�����</param>
/// <param name="maxPoint">��������</param>
/// <param name="negative">true: ����/false: ����</param>
/// <returns>���ú����</returns>
Point3Fs* cropBox(Point3F points[], const int length, const Point3F minPoint, const Point3F maxPoint, const bool negative)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();
	const Eigen::Vector4f& min = Eigen::Vector4f(minPoint.X, minPoint.Y, minPoint.Z, 1.0f);
	const Eigen::Vector4f& max = Eigen::Vector4f(maxPoint.X, maxPoint.Y, maxPoint.Z, 1.0f);

	//���������
	CropBox<PointXYZ> cropBox;
	cropBox.setInputCloud(sourceCloud);
	cropBox.setMin(min);
	cropBox.setMax(max);
	cropBox.setNegative(negative);
	cropBox.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// ͹������
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="contourPoints">�����㼯</param>
/// <param name="contourLength">�����㼯����</param>
/// <param name="dimensionsCount">ά����</param>
/// <returns>���ú����</returns>
Point3Fs* cropConvexHull(Point3F points[], int length, Point3F contourPoints[], int contourLength, int dimensionsCount)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//���嶥��
	PointCloud<PointXYZ>::Ptr vertices = std::make_shared<PointCloud<PointXYZ>>();
	for (int i = 0; i < contourLength; i++)
	{
		const Point3F& point3F = contourPoints[i];
		vertices->push_back(PointXYZ(point3F.X, point3F.Y, point3F.Z));
	}

	//����͹��
	PointCloud<PointXYZ>::Ptr hullCloud = std::make_shared<PointCloud<PointXYZ>>();
	vector<Vertices> hullIndices;
	ConvexHull<PointXYZ> convexHull;
	convexHull.setInputCloud(vertices);
	convexHull.setDimension(dimensionsCount);
	convexHull.reconstruct(*hullCloud, hullIndices);

	//͹���ü�
	CropHull<PointXYZ> cropHull;
	cropHull.setInputCloud(sourceCloud);
	cropHull.setHullCloud(hullCloud);
	cropHull.setHullIndices(hullIndices);
	cropHull.setDim(dimensionsCount);
	cropHull.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

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
Point3Fs* projectPlane(Point3F points[], const int length, const float a, const float b, const float c, const float d)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//����ƽ��ax + by +cz + d = 0
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(4);
	coefficients->values[0] = a;
	coefficients->values[1] = b;
	coefficients->values[2] = c;
	coefficients->values[3] = d;

	//Ͷ��ƽ��
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_PLANE);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

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
Point3Fs* projectLine(Point3F points[], const int length, const float a, const float b, const float c, const float d, const float e, const float f) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// ����ֱ��
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(6);
	coefficients->values[0] = a;
	coefficients->values[1] = b;
	coefficients->values[2] = c;
	coefficients->values[3] = d;
	coefficients->values[4] = e;
	coefficients->values[5] = f;

	// Ͷ��ֱ��
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_LINE);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// Ͷ��2DԲ
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="x">Բ��x����</param>
/// <param name="y">Բ��y����</param>
/// <param name="radius">�뾶</param>
/// <returns>Ͷ������</returns>
Point3Fs* projectCircle2D(Point3F points[], const int length, const float x, const float y, const float radius) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// ����2DԲ
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(3);
	coefficients->values[0] = x;
	coefficients->values[1] = y;
	coefficients->values[2] = radius;

	// Ͷ��2DԲ
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_CIRCLE2D);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

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
Point3Fs* projectCircle3D(Point3F points[], const int length, const float x, const float y, const float z, const float radius) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// ����3DԲ
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(4);
	coefficients->values[0] = x;
	coefficients->values[1] = y;
	coefficients->values[2] = z;
	coefficients->values[3] = radius;

	// Ͷ��3DԲ
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_CIRCLE3D);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

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
Point3Fs* projectSphere(Point3F points[], const int length, const float x, const float y, const float z, const float radius) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// ������
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(4);
	coefficients->values[0] = x;
	coefficients->values[1] = y;
	coefficients->values[2] = z;
	coefficients->values[3] = radius;

	// Ͷ����
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_SPHERE);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

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
Point3Fs* projectCylinder(Point3F points[], const int length, const float x, const float y, const float z, const float dx, const float dy, const float dz, const float radius) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// ����Բ��
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(7);
	coefficients->values[0] = x;
	coefficients->values[1] = y;
	coefficients->values[2] = z;
	coefficients->values[3] = dx;
	coefficients->values[4] = dy;
	coefficients->values[5] = dz;
	coefficients->values[6] = radius;

	// Ͷ��Բ��
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_CYLINDER);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

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
Point3Fs* projectCone(Point3F points[], const int length, const float x, const float y, const float z, const float dx, const float dy, const float dz, const float angle) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// ����Բ׶
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(7);
	coefficients->values[0] = x;
	coefficients->values[1] = y;
	coefficients->values[2] = z;
	coefficients->values[3] = dx;
	coefficients->values[4] = dy;
	coefficients->values[5] = dz;
	coefficients->values[6] = angle;

	// Ͷ��Բ׶
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_CONE);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

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
Point3Fs* projectTorus(Point3F points[], const int length, const float x, const float y, const float z, const float dx, const float dy, const float dz, const float radius, const float tube_radius) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// ����Բ��
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(7);
	coefficients->values[0] = x;
	coefficients->values[1] = y;
	coefficients->values[2] = z;
	coefficients->values[3] = dx;
	coefficients->values[4] = dy;
	coefficients->values[5] = dz;
	coefficients->values[6] = radius;
	coefficients->values[7] = tube_radius;

	// Ͷ��Բ��
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_TORUS);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

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
Point3Fs* projectParallelLine(Point3F points[], const int length, const float x, const float y, const float z) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// ����ƽ����
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(3);
	coefficients->values[0] = x;
	coefficients->values[1] = y;
	coefficients->values[2] = z;

	// Ͷ��ƽ����
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_PARALLEL_LINE);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// Ͷ�䴹ֱƽ��
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="x">������x����</param>
/// <param name="y">������y����</param>
/// <param name="z">������z����</param>
/// <returns>Ͷ������</returns>
Point3Fs* projectPerpendicularPlane(Point3F points[], const int length, const float x, const float y, const float z) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// ���崹ֱƽ��
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(3);
	coefficients->values[0] = x;
	coefficients->values[1] = y;
	coefficients->values[2] = z;

	// Ͷ�䴹ֱƽ��
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// Ͷ��ƽ���߼���
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="x">ƽ���߷���x����</param>
/// <param name="y">ƽ���߷���y����</param>
/// <param name="z">ƽ���߷���z����</param>
/// <returns>Ͷ������</returns>
Point3Fs* projectParallelLines(Point3F points[], const int length, const float x, const float y, const float z) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// ����ƽ���߼���
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(3);
	coefficients->values[0] = x;
	coefficients->values[1] = y;
	coefficients->values[2] = z;

	// Ͷ��ƽ���߼���
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_PARALLEL_LINES);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

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
Point3Fs* projectNormalPlane(Point3F points[], const int length, const float x, const float y, const float z, const float nx, const float ny, const float nz) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// ���巨��ƽ��
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(6);
	coefficients->values[0] = x;
	coefficients->values[1] = y;
	coefficients->values[2] = z;
	coefficients->values[3] = nx;
	coefficients->values[4] = ny;
	coefficients->values[5] = nz;

	// Ͷ�䷨��ƽ��
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_NORMAL_PLANE);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

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
Point3Fs* projectNormalSphere(Point3F points[], const int length, const float x, const float y, const float z, const float radius, const float nx, const float ny, const float nz) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// ���巨����
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(6);
	coefficients->values[0] = x;
	coefficients->values[1] = y;
	coefficients->values[2] = z;
	coefficients->values[3] = radius;
	coefficients->values[4] = nx;
	coefficients->values[5] = ny;
	coefficients->values[6] = nz;

	// Ͷ�䷨����
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_NORMAL_SPHERE);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

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
Point3Fs* projectRegistration(Point3F points[], const int length, const float tx, const float ty, const float tz, const float qx, const float qy, const float qz, const float qw) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// ����ע��ģ��
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(7);
	coefficients->values[0] = tx;
	coefficients->values[1] = ty;
	coefficients->values[2] = tz;
	coefficients->values[3] = qx;
	coefficients->values[4] = qy;
	coefficients->values[5] = qz;
	coefficients->values[6] = qw;

	// Ͷ��ע��ģ��
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_REGISTRATION);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

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
Point3Fs* projectRegistration2D(Point3F points[], const int length, const float tx, const float ty, const float tz, const float qx, const float qy, const float qz, const float qw) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// ����2Dע��ģ��
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(7);
	coefficients->values[0] = tx;
	coefficients->values[1] = ty;
	coefficients->values[2] = tz;
	coefficients->values[3] = qx;
	coefficients->values[4] = qy;
	coefficients->values[5] = qz;
	coefficients->values[6] = qw;

	// Ͷ��2Dע��ģ��
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_REGISTRATION_2D);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// Ͷ��ƽ��ƽ��
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="x">������x����</param>
/// <param name="y">������y����</param>
/// <param name="z">������z����</param>
/// <returns>Ͷ������</returns>
Point3Fs* projectParallelPlane(Point3F points[], const int length, const float x, const float y, const float z) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// ����ƽ��ƽ��
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(3);
	coefficients->values[0] = x;
	coefficients->values[1] = y;
	coefficients->values[2] = z;

	// Ͷ��ƽ��ƽ��
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// Ͷ��ƽ�з���ƽ��
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="x">������x����</param>
/// <param name="y">������y����</param>
/// <param name="z">������z����</param>
/// <returns>Ͷ������</returns>
Point3Fs* projectNormalParallelPlane(Point3F points[], const int length, const float x, const float y, const float z) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// ����ƽ�з���ƽ��
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(3);
	coefficients->values[0] = x;
	coefficients->values[1] = y;
	coefficients->values[2] = z;

	// Ͷ��ƽ�з���ƽ��
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

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
Point3Fs* projectStick(Point3F points[], const int length, const float x1, const float y1, const float z1, const float x2, const float y2, const float z2) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// �����״��
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(6);
	coefficients->values[0] = x1;
	coefficients->values[1] = y1;
	coefficients->values[2] = z1;
	coefficients->values[3] = x2;
	coefficients->values[4] = y2;
	coefficients->values[5] = z2;

	// Ͷ���״��
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_STICK);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

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
Point3Fs* projectEllipse3D(Point3F points[], const int length, const float x, const float y, const float z, const float a, const float b, const float c, const float d, const float e) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// ����3D��Բ
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(7);
	coefficients->values[0] = x;
	coefficients->values[1] = y;
	coefficients->values[2] = z;
	coefficients->values[3] = a;
	coefficients->values[4] = b;
	coefficients->values[5] = c;
	coefficients->values[6] = d;
	coefficients->values[7] = e;

	// Ͷ��3D��Բ
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_ELLIPSE3D);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// ��ȡ�߿�
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <returns>�߿����</returns>
Point3Fs* extractBorder(Point3F points[], const int length)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();
	const RangeImage::Ptr rangeImage = std::make_shared<RangeImage>();
	const PointCloud<BorderDescription>::Ptr borderDescriptions = std::make_shared<PointCloud<BorderDescription>>();

	//���ò���
	constexpr float angularResolution = 0.5f;
	constexpr float maxAngleWidth = 360.0f;
	constexpr float maxAngleHeight = 180.0f;
	const Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Affine3f::Identity());
	constexpr RangeImage::CoordinateFrame coordinateFrame = RangeImage::CAMERA_FRAME;
	constexpr float noiseLevel = 0.0f;
	constexpr float minRange = 0.0f;
	constexpr int borderSize = 0.0f;

	//�������ͼ
	rangeImage->createFromPointCloud(*sourceCloud, deg2rad(angularResolution), deg2rad(maxAngleWidth), deg2rad(maxAngleHeight), sensorPose, coordinateFrame, noiseLevel, minRange, borderSize);
	rangeImage->setUnseenToMaxRange();

	//����߿�
	RangeImageBorderExtractor borderExtractor(&*rangeImage);
	borderExtractor.compute(*borderDescriptions);

	//��ȡ�߿�
	const PointCloud<PointWithRange>::Ptr borderPoints = std::make_shared<PointCloud<PointWithRange>>();
	for (int i = 0; i < rangeImage->height; i++)
	{
		for (int j = 0; j < rangeImage->width; j++)
		{
			if (borderDescriptions->points[i * rangeImage->width + j].traits[BORDER_TRAIT__OBSTACLE_BORDER])
			{
				borderPoints->points.push_back(rangeImage->points[i * rangeImage->width + j]);
			}
		}
	}

	pcl::copyPointCloud(*borderPoints, *targetCloud);
	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

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
Point3Fs* extractBoundary(Point3F points[], const int length, const int normalK, const float featureRadius, const float angleThreshold, const int threadsCount)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();
	const PointCloud<Normal>::Ptr normals = std::make_shared<PointCloud<Normal>>();
	const PointCloud<Boundary>::Ptr boundaries = std::make_shared<PointCloud<Boundary>>();
	search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();

	//���㷨����
	NormalEstimationOMP<PointXYZ, Normal> normalEstimator;
	normalEstimator.setInputCloud(sourceCloud);
	normalEstimator.setSearchMethod(kdTree);
	normalEstimator.setKSearch(normalK);
	normalEstimator.setNumberOfThreads(threadsCount);
	normalEstimator.compute(*normals);

	//��ȡ�߽�
	BoundaryEstimation<PointXYZ, Normal, Boundary> boundaryComputer;
	boundaryComputer.setInputCloud(sourceCloud);
	boundaryComputer.setInputNormals(normals);
	boundaryComputer.setSearchMethod(kdTree);
	boundaryComputer.setRadiusSearch(featureRadius);
	boundaryComputer.setAngleThreshold(deg2rad(angleThreshold));
	boundaryComputer.compute(*boundaries);

	//�߽�ת������
	for (int i = 0; i < sourceCloud->points.size(); i++)
	{
		if (boundaries->points[i].boundary_point > 0)
		{
			targetCloud->push_back(sourceCloud->points[i]);
		}
	}

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}
