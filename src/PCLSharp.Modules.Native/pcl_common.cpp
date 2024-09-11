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
/// 估算质心
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <returns>质心坐标点</returns>
Point3F* estimateCentroid(Point3F points[], const int length)
{
	//加载点云
	const PointCloud<PointXYZ>::Ptr& cloud = pclsharp::toPointCloud(points, length);

	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloud, centroid);

	Point3F* point3F = new Point3F(centroid[0], centroid[1], centroid[2]);

	return point3F;
}

/// <summary>
/// 仿射变换
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="pose">位姿</param>
/// <returns>变换后点云</returns>
Point3Fs* affineTransform(Point3F points[], const int length, const Pose pose)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//创建仿射变换对象
	Eigen::Affine3f affine = Eigen::Affine3f::Identity();

	//平移
	affine.translation() << pose.X, pose.Y, pose.Z;

	//旋转
	const Eigen::AngleAxisf& rotationX = Eigen::AngleAxisf(pose.RX, Eigen::Vector3f::UnitX());
	const Eigen::AngleAxisf& rotationY = Eigen::AngleAxisf(pose.RY, Eigen::Vector3f::UnitY());
	const Eigen::AngleAxisf& rotationZ = Eigen::AngleAxisf(pose.RZ, Eigen::Vector3f::UnitZ());
	affine.rotate(rotationX);
	affine.rotate(rotationY);
	affine.rotate(rotationZ);

	//执行变换
	pcl::transformPointCloud(*sourceCloud, *targetCloud, affine);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 矩阵变换
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="matrixArray">矩阵数组(长度: 16)</param>
/// <returns>变换后点云</returns>
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

	//执行变换
	pcl::transformPointCloud(*sourceCloud, *targetCloud, rtMatrix);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 合并坐标点法向量
/// </summary>
/// <param name="points">点集</param>
/// <param name="normal3Fs">法向量集</param>
/// <param name="length">点集长度</param>
/// <returns>点集</returns>
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
/// 长方体剪裁
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="minPoint">最小坐标点</param>
/// <param name="maxPoint">最大坐标点</param>
/// <param name="negative">true: 剪裁/false: 保留</param>
/// <returns>剪裁后点云</returns>
Point3Fs* cropBox(Point3F points[], const int length, const Point3F minPoint, const Point3F maxPoint, const bool negative)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();
	const Eigen::Vector4f& min = Eigen::Vector4f(minPoint.X, minPoint.Y, minPoint.Z, 1.0f);
	const Eigen::Vector4f& max = Eigen::Vector4f(maxPoint.X, maxPoint.Y, maxPoint.Z, 1.0f);

	//长方体剪裁
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
/// 凸包剪裁
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="contourPoints">轮廓点集</param>
/// <param name="contourLength">轮廓点集长度</param>
/// <param name="dimensionsCount">维度数</param>
/// <returns>剪裁后点云</returns>
Point3Fs* cropConvexHull(Point3F points[], int length, Point3F contourPoints[], int contourLength, int dimensionsCount)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//定义顶点
	PointCloud<PointXYZ>::Ptr vertices = std::make_shared<PointCloud<PointXYZ>>();
	for (int i = 0; i < contourLength; i++)
	{
		const Point3F& point3F = contourPoints[i];
		vertices->push_back(PointXYZ(point3F.X, point3F.Y, point3F.Z));
	}

	//构造凸包
	PointCloud<PointXYZ>::Ptr hullCloud = std::make_shared<PointCloud<PointXYZ>>();
	vector<Vertices> hullIndices;
	ConvexHull<PointXYZ> convexHull;
	convexHull.setInputCloud(vertices);
	convexHull.setDimension(dimensionsCount);
	convexHull.reconstruct(*hullCloud, hullIndices);

	//凸包裁剪
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
/// 投射平面
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="a">平面方程系数a</param>
/// <param name="b">平面方程系数b</param>
/// <param name="c">平面方程系数c</param>
/// <param name="d">平面方程系数d</param>
/// <returns>投射后点云</returns>
/// <remarks>平面方程: ax + by +cz + d = 0</remarks>
Point3Fs* projectPlane(Point3F points[], const int length, const float a, const float b, const float c, const float d)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//定义平面ax + by +cz + d = 0
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(4);
	coefficients->values[0] = a;
	coefficients->values[1] = b;
	coefficients->values[2] = c;
	coefficients->values[3] = d;

	//投射平面
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_PLANE);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 投射线
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="a">线方程系数a</param>
/// <param name="b">线方程系数b</param>
/// <param name="c">线方程系数c</param>
/// <param name="d">线方程系数d</param>
/// <param name="e">线方程系数e</param>
/// <param name="f">线方程系数f</param>
/// <returns>投射后点云</returns>
/// <remarks>线方程: (x, y, z) = (a, b, c) + t(d, e, f)</remarks>
Point3Fs* projectLine(Point3F points[], const int length, const float a, const float b, const float c, const float d, const float e, const float f) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// 定义直线
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(6);
	coefficients->values[0] = a;
	coefficients->values[1] = b;
	coefficients->values[2] = c;
	coefficients->values[3] = d;
	coefficients->values[4] = e;
	coefficients->values[5] = f;

	// 投射直线
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_LINE);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 投射2D圆
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="x">圆心x坐标</param>
/// <param name="y">圆心y坐标</param>
/// <param name="radius">半径</param>
/// <returns>投射后点云</returns>
Point3Fs* projectCircle2D(Point3F points[], const int length, const float x, const float y, const float radius) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// 定义2D圆
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(3);
	coefficients->values[0] = x;
	coefficients->values[1] = y;
	coefficients->values[2] = radius;

	// 投射2D圆
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_CIRCLE2D);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 投射3D圆
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="x">圆心x坐标</param>
/// <param name="y">圆心y坐标</param>
/// <param name="z">圆心z坐标</param>
/// <param name="radius">半径</param>
/// <returns>投射后点云</returns>
Point3Fs* projectCircle3D(Point3F points[], const int length, const float x, const float y, const float z, const float radius) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// 定义3D圆
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(4);
	coefficients->values[0] = x;
	coefficients->values[1] = y;
	coefficients->values[2] = z;
	coefficients->values[3] = radius;

	// 投射3D圆
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_CIRCLE3D);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 投射球
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="x">球心x坐标</param>
/// <param name="y">球心y坐标</param>
/// <param name="z">球心z坐标</param>
/// <param name="radius">半径</param>
/// <returns>投射后点云</returns>
Point3Fs* projectSphere(Point3F points[], const int length, const float x, const float y, const float z, const float radius) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// 定义球
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(4);
	coefficients->values[0] = x;
	coefficients->values[1] = y;
	coefficients->values[2] = z;
	coefficients->values[3] = radius;

	// 投射球
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_SPHERE);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 投射圆柱
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="x">圆柱中心线起点x坐标</param>
/// <param name="y">圆柱中心线起点y坐标</param>
/// <param name="z">圆柱中心线起点z坐标</param>
/// <param name="dx">圆柱中心线方向x分量</param>
/// <param name="dy">圆柱中心线方向y分量</param>
/// <param name="dz">圆柱中心线方向z分量</param>
/// <param name="radius">半径</param>
/// <returns>投射后点云</returns>
Point3Fs* projectCylinder(Point3F points[], const int length, const float x, const float y, const float z, const float dx, const float dy, const float dz, const float radius) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// 定义圆柱
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(7);
	coefficients->values[0] = x;
	coefficients->values[1] = y;
	coefficients->values[2] = z;
	coefficients->values[3] = dx;
	coefficients->values[4] = dy;
	coefficients->values[5] = dz;
	coefficients->values[6] = radius;

	// 投射圆柱
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_CYLINDER);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 投射圆锥
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="x">圆锥顶点x坐标</param>
/// <param name="y">圆锥顶点y坐标</param>
/// <param name="z">圆锥顶点z坐标</param>
/// <param name="dx">圆锥方向x分量</param>
/// <param name="dy">圆锥方向y分量</param>
/// <param name="dz">圆锥方向z分量</param>
/// <param name="angle">圆锥角度</param>
/// <returns>投射后点云</returns>
Point3Fs* projectCone(Point3F points[], const int length, const float x, const float y, const float z, const float dx, const float dy, const float dz, const float angle) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// 定义圆锥
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(7);
	coefficients->values[0] = x;
	coefficients->values[1] = y;
	coefficients->values[2] = z;
	coefficients->values[3] = dx;
	coefficients->values[4] = dy;
	coefficients->values[5] = dz;
	coefficients->values[6] = angle;

	// 投射圆锥
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_CONE);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 投射圆环
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="x">圆环中心x坐标</param>
/// <param name="y">圆环中心y坐标</param>
/// <param name="z">圆环中心z坐标</param>
/// <param name="dx">圆环法向量x分量</param>
/// <param name="dy">圆环法向量y分量</param>
/// <param name="dz">圆环法向量z分量</param>
/// <param name="radius">圆环半径</param>
/// <param name="tube_radius">圆环管半径</param>
/// <returns>投射后点云</returns>
Point3Fs* projectTorus(Point3F points[], const int length, const float x, const float y, const float z, const float dx, const float dy, const float dz, const float radius, const float tube_radius) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// 定义圆环
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

	// 投射圆环
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_TORUS);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

// 其他模型的函数可以按照上面的模板进行类似的编写，根据模型的具体参数进行调整。
/// <summary>
/// 投射平行线
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="x">平行线方向x分量</param>
/// <param name="y">平行线方向y分量</param>
/// <param name="z">平行线方向z分量</param>
/// <returns>投射后点云</returns>
Point3Fs* projectParallelLine(Point3F points[], const int length, const float x, const float y, const float z) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// 定义平行线
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(3);
	coefficients->values[0] = x;
	coefficients->values[1] = y;
	coefficients->values[2] = z;

	// 投射平行线
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_PARALLEL_LINE);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 投射垂直平面
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="x">法向量x分量</param>
/// <param name="y">法向量y分量</param>
/// <param name="z">法向量z分量</param>
/// <returns>投射后点云</returns>
Point3Fs* projectPerpendicularPlane(Point3F points[], const int length, const float x, const float y, const float z) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// 定义垂直平面
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(3);
	coefficients->values[0] = x;
	coefficients->values[1] = y;
	coefficients->values[2] = z;

	// 投射垂直平面
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 投射平行线集合
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="x">平行线方向x分量</param>
/// <param name="y">平行线方向y分量</param>
/// <param name="z">平行线方向z分量</param>
/// <returns>投射后点云</returns>
Point3Fs* projectParallelLines(Point3F points[], const int length, const float x, const float y, const float z) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// 定义平行线集合
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(3);
	coefficients->values[0] = x;
	coefficients->values[1] = y;
	coefficients->values[2] = z;

	// 投射平行线集合
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_PARALLEL_LINES);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 投射法向平面
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="x">法向量x分量</param>
/// <param name="y">法向量y分量</param>
/// <param name="z">法向量z分量</param>
/// <param name="nx">法向量x分量</param>
/// <param name="ny">法向量y分量</param>
/// <param name="nz">法向量z分量</param>
/// <returns>投射后点云</returns>
Point3Fs* projectNormalPlane(Point3F points[], const int length, const float x, const float y, const float z, const float nx, const float ny, const float nz) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// 定义法向平面
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(6);
	coefficients->values[0] = x;
	coefficients->values[1] = y;
	coefficients->values[2] = z;
	coefficients->values[3] = nx;
	coefficients->values[4] = ny;
	coefficients->values[5] = nz;

	// 投射法向平面
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_NORMAL_PLANE);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 投射法向球
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="x">球心x坐标</param>
/// <param name="y">球心y坐标</param>
/// <param name="z">球心z坐标</param>
/// <param name="radius">半径</param>
/// <param name="nx">法向量x分量</param>
/// <param name="ny">法向量y分量</param>
/// <param name="nz">法向量z分量</param>
/// <returns>投射后点云</returns>
Point3Fs* projectNormalSphere(Point3F points[], const int length, const float x, const float y, const float z, const float radius, const float nx, const float ny, const float nz) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// 定义法向球
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(6);
	coefficients->values[0] = x;
	coefficients->values[1] = y;
	coefficients->values[2] = z;
	coefficients->values[3] = radius;
	coefficients->values[4] = nx;
	coefficients->values[5] = ny;
	coefficients->values[6] = nz;

	// 投射法向球
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_NORMAL_SPHERE);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 投射注册模型
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="tx">平移向量x分量</param>
/// <param name="ty">平移向量y分量</param>
/// <param="tz">平移向量z分量</param>
/// <param name="qx">四元数x分量</param>
/// <param name="qy">四元数y分量</param>
/// <param name="qz">四元数z分量</param>
/// <param name="qw">四元数w分量</param>
/// <returns>投射后点云</returns>
Point3Fs* projectRegistration(Point3F points[], const int length, const float tx, const float ty, const float tz, const float qx, const float qy, const float qz, const float qw) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// 定义注册模型
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(7);
	coefficients->values[0] = tx;
	coefficients->values[1] = ty;
	coefficients->values[2] = tz;
	coefficients->values[3] = qx;
	coefficients->values[4] = qy;
	coefficients->values[5] = qz;
	coefficients->values[6] = qw;

	// 投射注册模型
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_REGISTRATION);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 投射2D注册模型
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="tx">平移向量x分量</param>
/// <param name="ty">平移向量y分量</param>
/// <param name="tz">平移向量z分量</param>
/// <param name="qx">四元数x分量</param>
/// <param name="qy">四元数y分量</param>
/// <param name="qz">四元数z分量</param>
/// <param name="qw">四元数w分量</param>
/// <returns>投射后点云</returns>
Point3Fs* projectRegistration2D(Point3F points[], const int length, const float tx, const float ty, const float tz, const float qx, const float qy, const float qz, const float qw) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// 定义2D注册模型
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(7);
	coefficients->values[0] = tx;
	coefficients->values[1] = ty;
	coefficients->values[2] = tz;
	coefficients->values[3] = qx;
	coefficients->values[4] = qy;
	coefficients->values[5] = qz;
	coefficients->values[6] = qw;

	// 投射2D注册模型
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_REGISTRATION_2D);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 投射平行平面
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="x">法向量x分量</param>
/// <param name="y">法向量y分量</param>
/// <param name="z">法向量z分量</param>
/// <returns>投射后点云</returns>
Point3Fs* projectParallelPlane(Point3F points[], const int length, const float x, const float y, const float z) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// 定义平行平面
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(3);
	coefficients->values[0] = x;
	coefficients->values[1] = y;
	coefficients->values[2] = z;

	// 投射平行平面
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 投射平行法向平面
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="x">法向量x分量</param>
/// <param name="y">法向量y分量</param>
/// <param name="z">法向量z分量</param>
/// <returns>投射后点云</returns>
Point3Fs* projectNormalParallelPlane(Point3F points[], const int length, const float x, const float y, const float z) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// 定义平行法向平面
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(3);
	coefficients->values[0] = x;
	coefficients->values[1] = y;
	coefficients->values[2] = z;

	// 投射平行法向平面
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 投射棒状物
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="x1">棒状物起点x坐标</param>
/// <param name="y1">棒状物起点y坐标</param>
/// <param name="z1">棒状物起点z坐标</param>
/// <param name="x2">棒状物终点x坐标</param>
/// <param name="y2">棒状物终点y坐标</param>
/// <param name="z2">棒状物终点z坐标</param>
/// <returns>投射后点云</returns>
Point3Fs* projectStick(Point3F points[], const int length, const float x1, const float y1, const float z1, const float x2, const float y2, const float z2) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// 定义棒状物
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(6);
	coefficients->values[0] = x1;
	coefficients->values[1] = y1;
	coefficients->values[2] = z1;
	coefficients->values[3] = x2;
	coefficients->values[4] = y2;
	coefficients->values[5] = z2;

	// 投射棒状物
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_STICK);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 投射3D椭圆
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="x">椭圆中心x坐标</param>
/// <param name="y">椭圆中心y坐标</param>
/// <param name="z">椭圆中心z坐标</param>
/// <param name="a">长轴半径</param>
/// <param name="b">短轴半径</param>
/// <param name="c">椭圆法向量x分量</param>
/// <param name="d">椭圆法向量y分量</param>
/// <param name="e">椭圆法向量z分量</param>
/// <returns>投射后点云</returns>
Point3Fs* projectEllipse3D(Point3F points[], const int length, const float x, const float y, const float z, const float a, const float b, const float c, const float d, const float e) {
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	// 定义3D椭圆
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

	// 投射3D椭圆
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_ELLIPSE3D);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 提取边框
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <returns>边框点云</returns>
Point3Fs* extractBorder(Point3F points[], const int length)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();
	const RangeImage::Ptr rangeImage = std::make_shared<RangeImage>();
	const PointCloud<BorderDescription>::Ptr borderDescriptions = std::make_shared<PointCloud<BorderDescription>>();

	//设置参数
	constexpr float angularResolution = 0.5f;
	constexpr float maxAngleWidth = 360.0f;
	constexpr float maxAngleHeight = 180.0f;
	const Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Affine3f::Identity());
	constexpr RangeImage::CoordinateFrame coordinateFrame = RangeImage::CAMERA_FRAME;
	constexpr float noiseLevel = 0.0f;
	constexpr float minRange = 0.0f;
	constexpr int borderSize = 0.0f;

	//创建深度图
	rangeImage->createFromPointCloud(*sourceCloud, deg2rad(angularResolution), deg2rad(maxAngleWidth), deg2rad(maxAngleHeight), sensorPose, coordinateFrame, noiseLevel, minRange, borderSize);
	rangeImage->setUnseenToMaxRange();

	//计算边框
	RangeImageBorderExtractor borderExtractor(&*rangeImage);
	borderExtractor.compute(*borderDescriptions);

	//提取边框
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
/// 提取边界
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="normalK">法向量K</param>
/// <param name="featureRadius">特征半径</param>
/// <param name="angleThreshold">角度阈值</param>
/// <param name="threadsCount">线程数</param>
/// <returns>边界点云</returns>
Point3Fs* extractBoundary(Point3F points[], const int length, const int normalK, const float featureRadius, const float angleThreshold, const int threadsCount)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();
	const PointCloud<Normal>::Ptr normals = std::make_shared<PointCloud<Normal>>();
	const PointCloud<Boundary>::Ptr boundaries = std::make_shared<PointCloud<Boundary>>();
	search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();

	//计算法向量
	NormalEstimationOMP<PointXYZ, Normal> normalEstimator;
	normalEstimator.setInputCloud(sourceCloud);
	normalEstimator.setSearchMethod(kdTree);
	normalEstimator.setKSearch(normalK);
	normalEstimator.setNumberOfThreads(threadsCount);
	normalEstimator.compute(*normals);

	//提取边界
	BoundaryEstimation<PointXYZ, Normal, Boundary> boundaryComputer;
	boundaryComputer.setInputCloud(sourceCloud);
	boundaryComputer.setInputNormals(normals);
	boundaryComputer.setSearchMethod(kdTree);
	boundaryComputer.setRadiusSearch(featureRadius);
	boundaryComputer.setAngleThreshold(deg2rad(angleThreshold));
	boundaryComputer.compute(*boundaries);

	//边界转换点云
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
