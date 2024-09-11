using PCLSharp.Primitives.Constants;
using PCLSharp.Primitives.Models;
using System;
using System.Runtime.InteropServices;

namespace PCLSharp.Modules.Declarations
{
    /// <summary>
    /// 点云通用操作声明
    /// </summary>
    internal static class CommonNative
    {
        #region # 估算质心 —— static extern IntPtr EstimateCentroid(Point3F[] points...
        /// <summary>
        /// 估算质心
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <returns>质心坐标点</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "estimateCentroid")]
        public static extern IntPtr EstimateCentroid(Point3F[] points, int length);
        #endregion

        #region # 仿射变换 —— static extern IntPtr AffineTransform(Point3F[] points...
        /// <summary>
        /// 仿射变换
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="pose">位姿</param>
        /// <returns>变换后点云</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "affineTransform")]
        public static extern IntPtr AffineTransform(Point3F[] points, int length, Pose pose);
        #endregion

        #region # 矩阵变换 —— static extern IntPtr MatrixTransform(Point3F[] points...
        /// <summary>
        /// 矩阵变换
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="matrixArray">矩阵数组(长度: 16)</param>
        /// <returns>变换后点云</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "matrixTransform")]
        public static extern IntPtr MatrixTransform(Point3F[] points, int length, float[] matrixArray);
        #endregion

        #region # 合并坐标点法向量 —— static extern IntPtr MergePointsNormals(Point3F[] points...
        /// <summary>
        /// 合并坐标点法向量
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="normal3Fs">法向量集</param>
        /// <param name="length">点集长度</param>
        /// <returns>点集</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "mergePointsNormals")]
        public static extern IntPtr MergePointsNormals(Point3F[] points, Normal3F[] normal3Fs, int length);
        #endregion

        #region # 长方体剪裁 —— static extern IntPtr CropBox(Point3F[] points...
        /// <summary>
        /// 长方体剪裁
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="minPoint">最小坐标点</param>
        /// <param name="maxPoint">最大坐标点</param>
        /// <param name="negative">true: 剪裁/false: 保留</param>
        /// <returns>剪裁后点云</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "cropBox")]
        public static extern IntPtr CropBox(Point3F[] points, int length, Point3F minPoint, Point3F maxPoint, bool negative);
        #endregion

        #region # 凸包剪裁 —— static extern IntPtr CropConvexHull(Point3F[] points...
        /// <summary>
        /// 凸包剪裁
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="contourPoints">轮廓点集</param>
        /// <param name="contourLength">轮廓点集长度</param>
        /// <param name="dimensionsCount">维度数</param>
        /// <returns>剪裁后点云</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "cropConvexHull")]
        public static extern IntPtr CropConvexHull(Point3F[] points, int length, Point3F[] contourPoints, int contourLength, int dimensionsCount);
        #endregion

        #region # 投射平面 —— static extern IntPtr ProjectPlane(Point3F[] points...
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
        [DllImport(AssemblyNames.Modules, EntryPoint = "projectPlane")]
        public static extern IntPtr ProjectPlane(Point3F[] points, int length, float a, float b, float c, float d);
        #endregion

        #region # 投射线 —— static extern IntPtr ProjectLine(Point3F[] points...
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
        [DllImport(AssemblyNames.Modules, EntryPoint = "projectLine")]
        public static extern IntPtr ProjectLine(Point3F[] points, int length, float a, float b, float c, float d, float e, float f);
        #endregion

        #region # 投射2D圆 —— static extern IntPtr ProjectCircle2D(Point3F[] points...
        /// <summary>
        /// 投射2D圆
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="x">圆心x坐标</param>
        /// <param name="y">圆心y坐标</param>
        /// <param name="radius">半径</param>
        /// <returns>投射后点云</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "projectCircle2D")]
        public static extern IntPtr ProjectCircle2D(Point3F[] points, int length, float x, float y, float radius);
        #endregion

        #region # 投射3D圆 —— static extern IntPtr ProjectCircle3D(Point3F[] points...
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
        [DllImport(AssemblyNames.Modules, EntryPoint = "projectCircle3D")]
        public static extern IntPtr ProjectCircle3D(Point3F[] points, int length, float x, float y, float z, float radius);
        #endregion

        #region # 投射球 —— static extern IntPtr ProjectSphere(Point3F[] points...
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
        [DllImport(AssemblyNames.Modules, EntryPoint = "projectSphere")]
        public static extern IntPtr ProjectSphere(Point3F[] points, int length, float x, float y, float z, float radius);
        #endregion

        #region # 投射圆柱 —— static extern IntPtr ProjectCylinder(Point3F[] points...
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
        [DllImport(AssemblyNames.Modules, EntryPoint = "projectCylinder")]
        public static extern IntPtr ProjectCylinder(Point3F[] points, int length, float x, float y, float z, float dx, float dy, float dz, float radius);
        #endregion

        #region # 投射圆锥 —— static extern IntPtr ProjectCone(Point3F[] points...
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
        [DllImport(AssemblyNames.Modules, EntryPoint = "projectCone")]
        public static extern IntPtr ProjectCone(Point3F[] points, int length, float x, float y, float z, float dx, float dy, float dz, float angle);
        #endregion

        #region # 投射圆环 —— static extern IntPtr ProjectTorus(Point3F[] points...
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
        [DllImport(AssemblyNames.Modules, EntryPoint = "projectTorus")]
        public static extern IntPtr ProjectTorus(Point3F[] points, int length, float x, float y, float z, float dx, float dy, float dz, float radius, float tube_radius);
        #endregion

        #region # 投射平行线 —— static extern IntPtr ProjectParallelLine(Point3F[] points...
        /// <summary>
        /// 投射平行线
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="x">平行线方向x分量</param>
        /// <param name="y">平行线方向y分量</param>
        /// <param name="z">平行线方向z分量</param>
        /// <returns>投射后点云</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "projectParallelLine")]
        public static extern IntPtr ProjectParallelLine(Point3F[] points, int length, float x, float y, float z);
        #endregion

        #region # 投射垂直平面 —— static extern IntPtr ProjectPerpendicularPlane(Point3F[] points...
        /// <summary>
        /// 投射垂直平面
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="x">法向量x分量</param>
        /// <param name="y">法向量y分量</param>
        /// <param name="z">法向量z分量</param>
        /// <returns>投射后点云</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "projectPerpendicularPlane")]
        public static extern IntPtr ProjectPerpendicularPlane(Point3F[] points, int length, float x, float y, float z);
        #endregion

        #region # 投射平行线集合 —— static extern IntPtr ProjectParallelLines(Point3F[] points...
        /// <summary>
        /// 投射平行线集合
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="x">平行线方向x分量</param>
        /// <param name="y">平行线方向y分量</param>
        /// <param name="z">平行线方向z分量</param>
        /// <returns>投射后点云</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "projectParallelLines")]
        public static extern IntPtr ProjectParallelLines(Point3F[] points, int length, float x, float y, float z);
        #endregion

        #region # 投射法向平面 —— static extern IntPtr ProjectNormalPlane(Point3F[] points...
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
        [DllImport(AssemblyNames.Modules, EntryPoint = "projectNormalPlane")]
        public static extern IntPtr ProjectNormalPlane(Point3F[] points, int length, float x, float y, float z, float nx, float ny, float nz);
        #endregion

        #region # 投射法向球 —— static extern IntPtr ProjectNormalSphere(Point3F[] points...
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
        [DllImport(AssemblyNames.Modules, EntryPoint = "projectNormalSphere")]
        public static extern IntPtr ProjectNormalSphere(Point3F[] points, int length, float x, float y, float z, float radius, float nx, float ny, float nz);
        #endregion

        #region # 投射注册模型 —— static extern IntPtr ProjectRegistration(Point3F[] points...
        /// <summary>
        /// 投射注册模型
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
        [DllImport(AssemblyNames.Modules, EntryPoint = "projectRegistration")]
        public static extern IntPtr ProjectRegistration(Point3F[] points, int length, float tx, float ty, float tz, float qx, float qy, float qz, float qw);
        #endregion

        #region # 投射2D注册模型 —— static extern IntPtr ProjectRegistration2D(Point3F[] points...
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
        [DllImport(AssemblyNames.Modules, EntryPoint = "projectRegistration2D")]
        public static extern IntPtr ProjectRegistration2D(Point3F[] points, int length, float tx, float ty, float tz, float qx, float qy, float qz, float qw);
        #endregion

        #region # 投射平行平面 —— static extern IntPtr ProjectParallelPlane(Point3F[] points...
        /// <summary>
        /// 投射平行平面
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="x">法向量x分量</param>
        /// <param name="y">法向量y分量</param>
        /// <param name="z">法向量z分量</param>
        /// <returns>投射后点云</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "projectParallelPlane")]
        public static extern IntPtr ProjectParallelPlane(Point3F[] points, int length, float x, float y, float z);
        #endregion

        #region # 投射平行法向平面 —— static extern IntPtr ProjectNormalParallelPlane(Point3F[] points...
        /// <summary>
        /// 投射平行法向平面
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="x">法向量x分量</param>
        /// <param name="y">法向量y分量</param>
        /// <param name="z">法向量z分量</param>
        /// <returns>投射后点云</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "projectNormalParallelPlane")]
        public static extern IntPtr ProjectNormalParallelPlane(Point3F[] points, int length, float x, float y, float z);
        #endregion

        #region # 投射棒状物 —— static extern IntPtr ProjectStick(Point3F[] points...
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
        [DllImport(AssemblyNames.Modules, EntryPoint = "projectStick")]
        public static extern IntPtr ProjectStick(Point3F[] points, int length, float x1, float y1, float z1, float x2, float y2, float z2);
        #endregion

        #region # 投射3D椭圆 —— static extern IntPtr ProjectEllipse3D(Point3F[] points...
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
        [DllImport(AssemblyNames.Modules, EntryPoint = "projectEllipse3D")]
        public static extern IntPtr ProjectEllipse3D(Point3F[] points, int length, float x, float y, float z, float a, float b, float c, float d, float e);
        #endregion

        #region # 提取边框 —— static extern IntPtr ExtractBorder(Point3F[] points...
        /// <summary>
        /// 提取边框
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <returns>边框点云</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "extractBorder")]
        public static extern IntPtr ExtractBorder(Point3F[] points, int length);
        #endregion

        #region # 提取边界 —— static extern IntPtr ExtractBoundary(Point3F[] points...
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
        [DllImport(AssemblyNames.Modules, EntryPoint = "extractBoundary")]
        public static extern IntPtr ExtractBoundary(Point3F[] points, int length, int normalK, float featureRadius, float angleThreshold, int threadsCount);
        #endregion
    }
}
