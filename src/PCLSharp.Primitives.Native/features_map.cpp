#include "features_map.h"
using namespace std;
using namespace pcl;

/// <summary>
/// NARF����������ӳ��NARF����
/// </summary>
/// <param name="narf36Fs">NARF���������Ӽ�</param>
/// <param name="length">����</param>
/// <returns>NARF����</returns>
PointCloud<Narf36>::Ptr pclsharp::toPointCloud(Narf36F narf36Fs[], const int& length)
{
	const size_t& colsCount = 36;
	const PointCloud<Narf36>::Ptr& pointCloud = std::make_shared<PointCloud<Narf36>>();
	for (int rowIndex = 0; rowIndex < length; rowIndex++)
	{
		const Narf36F& narf36F = narf36Fs[rowIndex];
		Narf36 narf36 = Narf36(narf36F.X, narf36F.Y, narf36F.Z, narf36F.Roll, narf36F.Pitch, narf36F.Yaw);
		for (int colIndex = 0; colIndex < colsCount; colIndex++)
		{
			narf36.descriptor[colIndex] = narf36F.Features[colIndex];
		}
		pointCloud->push_back(narf36);
	}

	return pointCloud;
}

/// <summary>
/// NARF����ӳ��NARF����������
/// </summary>
/// <param name="pointCloud">NARF����</param>
/// <returns>NARF���������Ӽ�</returns>
Narf36Fs* pclsharp::toNarf36Fs(const PointCloud<Narf36>& pointCloud)
{
	const size_t& rowsCount = pointCloud.size();
	const size_t& colsCount = 36;
	Narf36F* descriptors = new Narf36F[rowsCount];
	for (int rowIndex = 0; rowIndex < rowsCount; rowIndex++)
	{
		const Narf36& narf36 = pointCloud.points[rowIndex];
		Narf36F narf36F = Narf36F(narf36.x, narf36.y, narf36.z, narf36.pitch, narf36.yaw, narf36.roll);
		for (int colIndex = 0; colIndex < colsCount; colIndex++)
		{
			const float& value = narf36.descriptor[colIndex];
			narf36F.Features[colIndex] = value;
		}
		descriptors[rowIndex] = narf36F;
	}

	Narf36Fs* narf36Fs = new Narf36Fs(descriptors, static_cast<int>(rowsCount));

	return narf36Fs;
}

/// <summary>
/// PFH����������ӳ��PFH����
/// </summary>
/// <param name="signature125Fs">PFH���������Ӽ�</param>
/// <param name="length">����</param>
/// <returns>PFH����</returns>
PointCloud<PFHSignature125>::Ptr pclsharp::toPointCloud(PFHSignature125F signature125Fs[], const int& length)
{
	const size_t& colsCount = 125;
	const PointCloud<PFHSignature125>::Ptr& pointCloud = std::make_shared<PointCloud<PFHSignature125>>();
	for (int rowIndex = 0; rowIndex < length; rowIndex++)
	{
		const PFHSignature125F& signature125F = signature125Fs[rowIndex];
		PFHSignature125 signature125 = PFHSignature125();
		for (int colIndex = 0; colIndex < colsCount; colIndex++)
		{
			signature125.histogram[colIndex] = signature125F.Features[colIndex];
		}
		pointCloud->push_back(signature125);
	}

	return pointCloud;
}

/// <summary>
/// PFH����ӳ��PFH����������
/// </summary>
/// <param name="pointCloud">PFH����</param>
/// <returns>PFH���������Ӽ�</returns>
PFHSignature125Fs* pclsharp::toPFHSignature125Fs(const PointCloud<PFHSignature125>& pointCloud)
{
	const size_t& rowsCount = pointCloud.size();
	const size_t& colsCount = 125;
	PFHSignature125F* descriptors = new PFHSignature125F[rowsCount];
	for (int rowIndex = 0; rowIndex < rowsCount; rowIndex++)
	{
		const PFHSignature125& signature125 = pointCloud.points[rowIndex];
		PFHSignature125F signature125F = PFHSignature125F();
		for (int colIndex = 0; colIndex < colsCount; colIndex++)
		{
			const float& value = signature125.histogram[colIndex];
			signature125F.Features[colIndex] = value;
		}
		descriptors[rowIndex] = signature125F;
	}

	PFHSignature125Fs* signature125Fs = new PFHSignature125Fs(descriptors, static_cast<int>(rowsCount));

	return signature125Fs;
}

/// <summary>
/// FPFH����������ӳ��FPFH����
/// </summary>
/// <param name="signature33Fs">FPFH���������Ӽ�</param>
/// <param name="length">����</param>
/// <returns>FPFH����</returns>
PointCloud<FPFHSignature33>::Ptr pclsharp::toPointCloud(FPFHSignature33F signature33Fs[], const int& length)
{
	const size_t& colsCount = 33;
	const PointCloud<FPFHSignature33>::Ptr& pointCloud = std::make_shared<PointCloud<FPFHSignature33>>();
	for (int rowIndex = 0; rowIndex < length; rowIndex++)
	{
		const FPFHSignature33F& signature33F = signature33Fs[rowIndex];
		FPFHSignature33 signature33 = FPFHSignature33();
		for (int colIndex = 0; colIndex < colsCount; colIndex++)
		{
			signature33.histogram[colIndex] = signature33F.Features[colIndex];
		}
		pointCloud->push_back(signature33);
	}

	return pointCloud;
}

/// <summary>
/// FPFH����ӳ��FPFH����������
/// </summary>
/// <param name="pointCloud">FPFH����</param>
/// <returns>FPFH���������Ӽ�</returns>
FPFHSignature33Fs* pclsharp::toFPFHSignature33Fs(const PointCloud<FPFHSignature33>& pointCloud)
{
	const size_t& rowsCount = pointCloud.size();
	const size_t& colsCount = 33;
	FPFHSignature33F* descriptors = new FPFHSignature33F[rowsCount];
	for (int rowIndex = 0; rowIndex < rowsCount; rowIndex++)
	{
		const FPFHSignature33& signature33 = pointCloud.points[rowIndex];
		FPFHSignature33F signature33F = FPFHSignature33F();
		for (int colIndex = 0; colIndex < colsCount; colIndex++)
		{
			const float& value = signature33.histogram[colIndex];
			signature33F.Features[colIndex] = value;
		}
		descriptors[rowIndex] = signature33F;
	}

	FPFHSignature33Fs* signature33Fs = new FPFHSignature33Fs(descriptors, static_cast<int>(rowsCount));

	return signature33Fs;
}

/// <summary>
/// 3DSC����������ӳ��3DSC����
/// </summary>
/// <param name="shapeContext1980Fs">3DSC���������Ӽ�</param>
/// <param name="length">����</param>
/// <returns>3DSC����</returns>
PointCloud<ShapeContext1980>::Ptr pclsharp::toPointCloud(ShapeContext1980F shapeContext1980Fs[], const int& length)
{
	const size_t& colsCount = 1980;
	const PointCloud<ShapeContext1980>::Ptr& pointCloud = std::make_shared<PointCloud<ShapeContext1980>>();
	for (int rowIndex = 0; rowIndex < length; rowIndex++)
	{
		const ShapeContext1980F& shapeContext1980F = shapeContext1980Fs[rowIndex];
		ShapeContext1980 shapeContext1980 = ShapeContext1980();
		for (int i = 0; i < 9; i++)
		{
			shapeContext1980.rf[i] = shapeContext1980F.RF[i];
		}
		for (int colIndex = 0; colIndex < colsCount; colIndex++)
		{
			shapeContext1980.descriptor[colIndex] = shapeContext1980F.Features[colIndex];
		}
		pointCloud->push_back(shapeContext1980);
	}

	return pointCloud;
}

/// <summary>
/// 3DSC����ӳ��3DSC����������
/// </summary>
/// <param name="pointCloud">3DSC����</param>
/// <returns>3DSC���������Ӽ�</returns>
ShapeContext1980Fs* pclsharp::toShapeContext1980Fs(const PointCloud<ShapeContext1980>& pointCloud)
{
	const size_t& rowsCount = pointCloud.size();
	const size_t& colsCount = 1980;
	ShapeContext1980F* descriptors = new ShapeContext1980F[rowsCount];
	for (int rowIndex = 0; rowIndex < rowsCount; rowIndex++)
	{
		const ShapeContext1980& shapeContext1980 = pointCloud.points[rowIndex];
		ShapeContext1980F shapeContext1980F = ShapeContext1980F();
		for (int i = 0; i < 9; i++)
		{
			shapeContext1980F.RF[i] = shapeContext1980.rf[i];
		}
		for (int colIndex = 0; colIndex < colsCount; colIndex++)
		{
			shapeContext1980F.Features[colIndex] = shapeContext1980.descriptor[colIndex];
		}
		descriptors[rowIndex] = shapeContext1980F;
	}

	ShapeContext1980Fs* shapeContext1980Fs = new ShapeContext1980Fs(descriptors, static_cast<int>(rowsCount));

	return shapeContext1980Fs;
}

/// <summary>
/// SHOT����������ӳ��SHOT����
/// </summary>
/// <param name="shot352Fs">SHOT���������Ӽ�</param>
/// <param name="length">����</param>
/// <returns>SHOT����</returns>
PointCloud<SHOT352>::Ptr pclsharp::toPointCloud(Shot352F shot352Fs[], const int& length)
{
	const size_t& colsCount = 352;
	const PointCloud<SHOT352>::Ptr& pointCloud = std::make_shared<PointCloud<SHOT352>>();
	for (int rowIndex = 0; rowIndex < length; rowIndex++)
	{
		const Shot352F& shot352F = shot352Fs[rowIndex];
		SHOT352 shot352 = SHOT352();
		for (int i = 0; i < 9; i++)
		{
			shot352.rf[i] = shot352F.RF[i];
		}
		for (int colIndex = 0; colIndex < colsCount; colIndex++)
		{
			shot352.descriptor[colIndex] = shot352F.Features[colIndex];
		}
		pointCloud->push_back(shot352);
	}

	return pointCloud;
}

/// <summary>
/// SHOT����ӳ��SHOT����������
/// </summary>
/// <param name="pointCloud">SHOT����</param>
/// <returns>SHOT���������Ӽ�</returns>
Shot352Fs* pclsharp::toShot352Fs(const PointCloud<SHOT352>& pointCloud)
{
	const size_t& rowsCount = pointCloud.size();
	const size_t& colsCount = 352;
	Shot352F* descriptors = new Shot352F[rowsCount];
	for (int rowIndex = 0; rowIndex < rowsCount; rowIndex++)
	{
		const SHOT352& shot352 = pointCloud.points[rowIndex];
		Shot352F shot352F = Shot352F();
		for (int i = 0; i < 9; i++)
		{
			shot352F.RF[i] = shot352.rf[i];
		}
		for (int colIndex = 0; colIndex < colsCount; colIndex++)
		{
			shot352F.Features[colIndex] = shot352.descriptor[colIndex];
		}
		descriptors[rowIndex] = shot352F;
	}

	Shot352Fs* shot352Fs = new Shot352Fs(descriptors, static_cast<int>(rowsCount));

	return shot352Fs;
}
