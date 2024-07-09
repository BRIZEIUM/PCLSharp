#pragma once
#ifdef _WIN32
#define EXPORT_CPP extern __declspec(dllexport)
#elif __linux__
#define EXPORT_CPP extern
#endif
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "narf36f.h"
#include "narf36fs.h"
#include "pfh_signature125f.h"
#include "pfh_signature125fs.h"
#include "fpfh_signature33f.h"
#include "fpfh_signature33fs.h"
#include "shape_context1980f.h"
#include "shape_context1980fs.h"
#include "shot352f.h"
#include "shot352fs.h"

namespace pclsharp
{
	/// <summary>
	/// NARF����������ӳ��NARF����
	/// </summary>
	/// <param name="narf36Fs">NARF���������Ӽ�</param>
	/// <param name="length">����</param>
	/// <returns>NARF����</returns>
	EXPORT_CPP pcl::PointCloud<pcl::Narf36>::Ptr toPointCloud(Narf36F narf36Fs[], const int& length);

	/// <summary>
	/// NARF����ӳ��NARF����������
	/// </summary>
	/// <param name="pointCloud">NARF����</param>
	/// <returns>NARF���������Ӽ�</returns>
	EXPORT_CPP Narf36Fs* toNarf36Fs(const pcl::PointCloud<pcl::Narf36>& pointCloud);

	/// <summary>
	/// PFH����������ӳ��PFH����
	/// </summary>
	/// <param name="signature125Fs">PFH���������Ӽ�</param>
	/// <param name="length">����</param>
	/// <returns>PFH����</returns>
	EXPORT_CPP pcl::PointCloud<pcl::PFHSignature125>::Ptr toPointCloud(PFHSignature125F signature125Fs[], const int& length);

	/// <summary>
	/// PFH����ӳ��PFH����������
	/// </summary>
	/// <param name="pointCloud">PFH����</param>
	/// <returns>PFH���������Ӽ�</returns>
	EXPORT_CPP PFHSignature125Fs* toPFHSignature125Fs(const pcl::PointCloud<pcl::PFHSignature125>& pointCloud);

	/// <summary>
	/// FPFH����������ӳ��FPFH����
	/// </summary>
	/// <param name="signature33Fs">FPFH���������Ӽ�</param>
	/// <param name="length">����</param>
	/// <returns>FPFH����</returns>
	EXPORT_CPP pcl::PointCloud<pcl::FPFHSignature33>::Ptr toPointCloud(FPFHSignature33F signature33Fs[], const int& length);

	/// <summary>
	/// FPFH����ӳ��FPFH����������
	/// </summary>
	/// <param name="pointCloud">FPFH����</param>
	/// <returns>FPFH���������Ӽ�</returns>
	EXPORT_CPP FPFHSignature33Fs* toFPFHSignature33Fs(const pcl::PointCloud<pcl::FPFHSignature33>& pointCloud);

	/// <summary>
	/// 3DSC����������ӳ��3DSC����
	/// </summary>
	/// <param name="shapeContext1980Fs">3DSC���������Ӽ�</param>
	/// <param name="length">����</param>
	/// <returns>3DSC����</returns>
	EXPORT_CPP pcl::PointCloud<pcl::ShapeContext1980>::Ptr toPointCloud(ShapeContext1980F shapeContext1980Fs[], const int& length);

	/// <summary>
	/// 3DSC����ӳ��3DSC����������
	/// </summary>
	/// <param name="pointCloud">3DSC����</param>
	/// <returns>3DSC���������Ӽ�</returns>
	EXPORT_CPP ShapeContext1980Fs* toShapeContext1980Fs(const pcl::PointCloud<pcl::ShapeContext1980>& pointCloud);

	/// <summary>
	/// SHOT����������ӳ��SHOT����
	/// </summary>
	/// <param name="shot352Fs">SHOT���������Ӽ�</param>
	/// <param name="length">����</param>
	/// <returns>SHOT����</returns>
	EXPORT_CPP pcl::PointCloud<pcl::SHOT352>::Ptr toPointCloud(Shot352F shot352Fs[], const int& length);

	/// <summary>
	/// SHOT����ӳ��SHOT����������
	/// </summary>
	/// <param name="pointCloud">SHOT����</param>
	/// <returns>SHOT���������Ӽ�</returns>
	EXPORT_CPP Shot352Fs* toShot352Fs(const pcl::PointCloud<pcl::SHOT352>& pointCloud);
}
