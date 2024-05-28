#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
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
	const PointCloud<PointXYZ>::Ptr sourceCloud = pclsharp::toPointCloud(points, length);
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
