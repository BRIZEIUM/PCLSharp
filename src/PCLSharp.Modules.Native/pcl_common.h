#pragma once
#ifdef _WIN32
#define EXPORT_C extern "C" __declspec(dllexport)
#define CALLING_MODE _cdecl
#elif __linux__
#define EXPORT_C extern "C"
#define CALLING_MODE __attribute__((__cdecl__))
#endif
#include <point3f.h>
#include <point3fs.h>
#include <pose.h>

/// <summary>
/// 估算质心
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <returns>质心坐标点</returns>
EXPORT_C Point3F* CALLING_MODE estimateCentroid(Point3F points[], int length);

/// <summary>
/// 仿射变换
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="pose">位姿</param>
/// <returns>变换后点云</returns>
EXPORT_C Point3Fs* CALLING_MODE affineTransform(Point3F points[], int length, Pose pose);

/// <summary>
/// 矩阵变换
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="matrixArray">矩阵数组(长度: 16)</param>
/// <returns>变换后点云</returns>
EXPORT_C Point3Fs* CALLING_MODE matrixTransform(Point3F points[], int length, float matrixArray[]);

/// <summary>
/// 合并坐标点法向量
/// </summary>
/// <param name="points">点集</param>
/// <param name="normal3Fs">法向量集</param>
/// <param name="length">点集长度</param>
/// <returns>点集</returns>
EXPORT_C Point3Normal3s* CALLING_MODE mergePointsNormals(Point3F points[], Normal3F normal3Fs[], int length);

/// <summary>
/// 长方体剪裁
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="minPoint">最小坐标点</param>
/// <param name="maxPoint">最大坐标点</param>
/// <param name="negative">true: 剪裁/false: 保留</param>
/// <returns>剪裁后点云</returns>
EXPORT_C Point3Fs* CALLING_MODE cropBox(Point3F points[], int length, Point3F minPoint, Point3F maxPoint, bool negative);

/// <summary>
/// 凸包剪裁
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="contourPoints">轮廓点集</param>
/// <param name="contourLength">轮廓点集长度</param>
/// <param name="dimensionsCount">维度数</param>
/// <returns>剪裁后点云</returns>
EXPORT_C Point3Fs* CALLING_MODE cropConvexHull(Point3F points[], int length, Point3F contourPoints[], int contourLength, int dimensionsCount);

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
EXPORT_C Point3Fs* CALLING_MODE projectPlane(Point3F points[], int length, float a, float b, float c, float d);

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
EXPORT_C Point3Fs* CALLING_MODE projectLine(Point3F points[], int length, float a, float b, float c, float d, float e, float f);

/// <summary>
/// 投射2D圆
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="x">圆心x坐标</param>
/// <param name="y">圆心y坐标</param>
/// <param name="radius">半径</param>
/// <returns>投射后点云</returns>
EXPORT_C Point3Fs* CALLING_MODE projectCircle2D(Point3F points[], int length, float x, float y, float radius);

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
EXPORT_C Point3Fs* CALLING_MODE projectCircle3D(Point3F points[], int length, float x, float y, float z, float radius);

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
EXPORT_C Point3Fs* CALLING_MODE projectSphere(Point3F points[], int length, float x, float y, float z, float radius);

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
EXPORT_C Point3Fs* CALLING_MODE projectCylinder(Point3F points[], int length, float x, float y, float z, float dx, float dy, float dz, float radius);

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
EXPORT_C Point3Fs* CALLING_MODE projectCone(Point3F points[], int length, float x, float y, float z, float dx, float dy, float dz, float angle);

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
EXPORT_C Point3Fs* CALLING_MODE projectTorus(Point3F points[], int length, float x, float y, float z, float dx, float dy, float dz, float radius, float tube_radius);

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
EXPORT_C Point3Fs* CALLING_MODE projectParallelLine(Point3F points[], int length, float x, float y, float z);

/// <summary>
/// 投射垂直平面
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="x">法向量x分量</param>
/// <param name="y">法向量y分量</param>
/// <param name="z">法向量z分量</param>
/// <returns>投射后点云</returns>
EXPORT_C Point3Fs* CALLING_MODE projectPerpendicularPlane(Point3F points[], int length, float x, float y, float z);

/// <summary>
/// 投射平行线集合
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="x">平行线方向x分量</param>
/// <param name="y">平行线方向y分量</param>
/// <param name="z">平行线方向z分量</param>
/// <returns>投射后点云</returns>
EXPORT_C Point3Fs* CALLING_MODE projectParallelLines(Point3F points[], int length, float x, float y, float z);

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
EXPORT_C Point3Fs* CALLING_MODE projectNormalPlane(Point3F points[], int length, float x, float y, float z, float nx, float ny, float nz);

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
EXPORT_C Point3Fs* CALLING_MODE projectNormalSphere(Point3F points[], int length, float x, float y, float z, float radius, float nx, float ny, float nz);

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
EXPORT_C Point3Fs* CALLING_MODE projectRegistration(Point3F points[], int length, float tx, float ty, float tz, float qx, float qy, float qz, float qw);

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
EXPORT_C Point3Fs* CALLING_MODE projectRegistration2D(Point3F points[], int length, float tx, float ty, float tz, float qx, float qy, float qz, float qw);

/// <summary>
/// 投射平行平面
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="x">法向量x分量</param>
/// <param name="y">法向量y分量</param>
/// <param name="z">法向量z分量</param>
/// <returns>投射后点云</returns>
EXPORT_C Point3Fs* CALLING_MODE projectParallelPlane(Point3F points[], int length, float x, float y, float z);

/// <summary>
/// 投射平行法向平面
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="x">法向量x分量</param>
/// <param name="y">法向量y分量</param>
/// <param name="z">法向量z分量</param>
/// <returns>投射后点云</returns>
EXPORT_C Point3Fs* CALLING_MODE projectNormalParallelPlane(Point3F points[], int length, float x, float y, float z);

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
EXPORT_C Point3Fs* CALLING_MODE projectStick(Point3F points[], int length, float x1, float y1, float z1, float x2, float y2, float z2);

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
EXPORT_C Point3Fs* CALLING_MODE projectEllipse3D(Point3F points[], int length, float x, float y, float z, float a, float b, float c, float d, float e);

/// <summary>
/// 提取边框
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <returns>边框点云</returns>
EXPORT_C Point3Fs* CALLING_MODE extractBorder(Point3F points[], int length);

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
EXPORT_C Point3Fs* CALLING_MODE extractBoundary(Point3F points[], int length, int normalK, float featureRadius, float angleThreshold, int threadsCount);
