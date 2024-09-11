using PCLSharp.Modules.Declarations;
using PCLSharp.Modules.Interfaces;
using PCLSharp.Primitives.Extensions;
using PCLSharp.Primitives.Models;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;

namespace PCLSharp.Modules.Implements
{
    /// <summary>
    /// 点云通用操作实现
    /// </summary>
    public class CloudCommon : ICloudCommon
    {
        #region # 估算质心 —— Point3F EstimateCentroid(IEnumerable<Point3F> points)
        /// <summary>
        /// 估算质心
        /// </summary>
        /// <param name="points">点集</param>
        /// <returns>质心坐标点</returns>
        public Point3F EstimateCentroid(IEnumerable<Point3F> points)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return new Point3F();
            }

            #endregion

            IntPtr pointer = CommonNative.EstimateCentroid(points_, points_.Length);
            Point3F centroid = Marshal.PtrToStructure<Point3F>(pointer);
            DisposeNative.DisposePoint3F(pointer);

            return centroid;
        }
        #endregion

        #region # 仿射变换 —— Point3F[] AffineTransform(IEnumerable<Point3F> points, Pose pose)
        /// <summary>
        /// 仿射变换
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="pose">位姿</param>
        /// <returns>变换后点云</returns>
        public Point3F[] AffineTransform(IEnumerable<Point3F> points, Pose pose)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = CommonNative.AffineTransform(points_, points_.Length, pose);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] transformedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return transformedPoints;
        }
        #endregion

        #region # 矩阵变换 —— Point3F[] MatrixTransform(IEnumerable<Point3F> points...
        /// <summary>
        /// 矩阵变换
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="matrixArray">矩阵数组(长度: 16)</param>
        /// <returns>变换后点云</returns>
        public Point3F[] MatrixTransform(IEnumerable<Point3F> points, float[] matrixArray)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = CommonNative.MatrixTransform(points_, points_.Length, matrixArray);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] transformedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return transformedPoints;
        }
        #endregion

        #region # 合并坐标点法向量 —— Point3Normal3[] MergePointsNormals(IEnumerable<Point3F> points...
        /// <summary>
        /// 合并坐标点法向量
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="normals">法向量集</param>
        /// <returns>点集</returns>
        public Point3Normal3[] MergePointsNormals(IEnumerable<Point3F> points, IEnumerable<Normal3F> normals)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();
            Normal3F[] normals_ = normals?.ToArray() ?? Array.Empty<Normal3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3Normal3>();
            }
            if (!normals_.Any())
            {
                return Array.Empty<Point3Normal3>();
            }
            if (points_.Length != normals_.Length)
            {
                throw new InvalidOperationException("点集与法向量集长度不一致！");
            }

            #endregion

            IntPtr pointer = CommonNative.MergePointsNormals(points_, normals_, points_.Length);
            Point3Normal3s point3Normal3s = Marshal.PtrToStructure<Point3Normal3s>(pointer);
            Point3Normal3[] pointNormals = point3Normal3s.Recover();
            DisposeNative.DisposePoint3Normal3s(pointer);

            return pointNormals;
        }
        #endregion

        #region # 长方体剪裁 —— Point3F[] CropBox(IEnumerable<Point3F> points...
        /// <summary>
        /// 长方体剪裁
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="minPoint">最小坐标点</param>
        /// <param name="maxPoint">最大坐标点</param>
        /// <param name="negative">true: 剪裁/false: 保留</param>
        /// <returns>剪裁后点云</returns>
        public Point3F[] CropBox(IEnumerable<Point3F> points, Point3F minPoint, Point3F maxPoint, bool negative)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = CommonNative.CropBox(points_, points_.Length, minPoint, maxPoint, negative);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] croppedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return croppedPoints;
        }
        #endregion

        #region # 凸包剪裁 —— Point3F[] CropConvexHull(IEnumerable<Point3F> points...
        /// <summary>
        /// 凸包剪裁
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="contourPoints">轮廓点集</param>
        /// <param name="dimensionsCount">维度数</param>
        /// <returns>剪裁后点云</returns>
        public Point3F[] CropConvexHull(IEnumerable<Point3F> points, IEnumerable<Point3F> contourPoints, int dimensionsCount)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();
            Point3F[] contourPoints_ = contourPoints?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }
            if (!contourPoints_.Any())
            {
                return points_;
            }

            #endregion

            IntPtr pointer = CommonNative.CropConvexHull(points_, points_.Length, contourPoints_, contourPoints_.Length, dimensionsCount);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] croppedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return croppedPoints;
        }
        #endregion

        #region # 投射平面 —— Point3F[] ProjectPlane(IEnumerable<Point3F> points...
        /// <summary>
        /// 投射平面
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="a">平面方程系数a</param>
        /// <param name="b">平面方程系数b</param>
        /// <param name="c">平面方程系数c</param>
        /// <param name="d">平面方程系数d</param>
        /// <returns>投射后点云</returns>
        /// <remarks>平面方程: ax + by +cz + d = 0</remarks>
        public Point3F[] ProjectPlane(IEnumerable<Point3F> points, float a, float b, float c, float d)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = CommonNative.ProjectPlane(points_, points_.Length, a, b, c, d);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] projectedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return projectedPoints;
        }
        #endregion

        #region # 投射线 —— Point3F[] ProjectLine(IEnumerable<Point3F> points...
        /// <summary>
        /// 投射线
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="a">线方程系数a</param>
        /// <param name="b">线方程系数b</param>
        /// <param name="c">线方程系数c</param>
        /// <param name="d">线方程系数d</param>
        /// <param name="e">线方程系数e</param>
        /// <param name="f">线方程系数f</param>
        /// <returns>投射后点云</returns>
        /// <remarks>线方程: (x, y, z) = (a, b, c) + t(d, e, f)</remarks>
        public Point3F[] ProjectLine(IEnumerable<Point3F> points, float a, float b, float c, float d, float e, float f)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证
            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }
            #endregion

            IntPtr pointer = CommonNative.ProjectLine(points_, points_.Length, a, b, c, d, e, f);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] projectedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return projectedPoints;
        }
        #endregion

        #region # 投射2D圆 —— Point3F[] ProjectCircle2D(IEnumerable<Point3F> points...
        /// <summary>
        /// 投射2D圆
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="x">圆心x坐标</param>
        /// <param name="y">圆心y坐标</param>
        /// <param name="radius">半径</param>
        /// <returns>投射后点云</returns>
        public Point3F[] ProjectCircle2D(IEnumerable<Point3F> points, float x, float y, float radius)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证
            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }
            #endregion

            IntPtr pointer = CommonNative.ProjectCircle2D(points_, points_.Length, x, y, radius);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] projectedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return projectedPoints;
        }
        #endregion

        #region # 投射3D圆 —— Point3F[] ProjectCircle3D(IEnumerable<Point3F> points...
        /// <summary>
        /// 投射3D圆
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="x">圆心x坐标</param>
        /// <param name="y">圆心y坐标</param>
        /// <param name="z">圆心z坐标</param>
        /// <param name="radius">半径</param>
        /// <returns>投射后点云</returns>
        public Point3F[] ProjectCircle3D(IEnumerable<Point3F> points, float x, float y, float z, float radius)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证
            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }
            #endregion

            IntPtr pointer = CommonNative.ProjectCircle3D(points_, points_.Length, x, y, z, radius);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] projectedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return projectedPoints;
        }
        #endregion

        #region # 投射球 —— Point3F[] ProjectSphere(IEnumerable<Point3F> points...
        /// <summary>
        /// 投射球
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="x">球心x坐标</param>
        /// <param name="y">球心y坐标</param>
        /// <param name="z">球心z坐标</param>
        /// <param name="radius">半径</param>
        /// <returns>投射后点云</returns>
        public Point3F[] ProjectSphere(IEnumerable<Point3F> points, float x, float y, float z, float radius)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证
            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }
            #endregion

            IntPtr pointer = CommonNative.ProjectSphere(points_, points_.Length, x, y, z, radius);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] projectedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return projectedPoints;
        }
        #endregion

        #region # 投射圆柱 —— Point3F[] ProjectCylinder(IEnumerable<Point3F> points...
        /// <summary>
        /// 投射圆柱
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="x">圆柱中心线起点x坐标</param>
        /// <param name="y">圆柱中心线起点y坐标</param>
        /// <param name="z">圆柱中心线起点z坐标</param>
        /// <param name="dx">圆柱中心线方向x分量</param>
        /// <param name="dy">圆柱中心线方向y分量</param>
        /// <param name="dz">圆柱中心线方向z分量</param>
        /// <param name="radius">半径</param>
        /// <returns>投射后点云</returns>
        public Point3F[] ProjectCylinder(IEnumerable<Point3F> points, float x, float y, float z, float dx, float dy, float dz, float radius)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证
            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }
            #endregion

            IntPtr pointer = CommonNative.ProjectCylinder(points_, points_.Length, x, y, z, dx, dy, dz, radius);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] projectedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return projectedPoints;
        }
        #endregion

        #region # 投射圆锥 —— Point3F[] ProjectCone(IEnumerable<Point3F> points...
        /// <summary>
        /// 投射圆锥
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="x">圆锥顶点x坐标</param>
        /// <param name="y">圆锥顶点y坐标</param>
        /// <param name="z">圆锥顶点z坐标</param>
        /// <param name="dx">圆锥方向x分量</param>
        /// <param name="dy">圆锥方向y分量</param>
        /// <param name="dz">圆锥方向z分量</param>
        /// <param name="angle">圆锥角度</param>
        /// <returns>投射后点云</returns>
        public Point3F[] ProjectCone(IEnumerable<Point3F> points, float x, float y, float z, float dx, float dy, float dz, float angle)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证
            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }
            #endregion

            IntPtr pointer = CommonNative.ProjectCone(points_, points_.Length, x, y, z, dx, dy, dz, angle);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] projectedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return projectedPoints;
        }
        #endregion

        #region # 投射圆环 —— Point3F[] ProjectTorus(IEnumerable<Point3F> points...
        /// <summary>
        /// 投射圆环
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="x">圆环中心x坐标</param>
        /// <param name="y">圆环中心y坐标</param>
        /// <param name="z">圆环中心z坐标</param>
        /// <param name="dx">圆环法向量x分量</param>
        /// <param name="dy">圆环法向量y分量</param>
        /// <param name="dz">圆环法向量z分量</param>
        /// <param name="radius">圆环半径</param>
        /// <param name="tube_radius">圆环管半径</param>
        /// <returns>投射后点云</returns>
        public Point3F[] ProjectTorus(IEnumerable<Point3F> points, float x, float y, float z, float dx, float dy, float dz, float radius, float tube_radius)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证
            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }
            #endregion

            IntPtr pointer = CommonNative.ProjectTorus(points_, points_.Length, x, y, z, dx, dy, dz, radius, tube_radius);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] projectedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return projectedPoints;
        }
        #endregion

        #region # 投射平行线 —— Point3F[] ProjectParallelLine(IEnumerable<Point3F> points...
        /// <summary>
        /// 投射平行线
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="x">平行线方向x分量</param>
        /// <param name="y">平行线方向y分量</param>
        /// <param name="z">平行线方向z分量</param>
        /// <returns>投射后点云</returns>
        public Point3F[] ProjectParallelLine(IEnumerable<Point3F> points, float x, float y, float z)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证
            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }
            #endregion

            IntPtr pointer = CommonNative.ProjectParallelLine(points_, points_.Length, x, y, z);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] projectedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return projectedPoints;
        }
        #endregion

        #region # 投射垂直平面 —— Point3F[] ProjectPerpendicularPlane(IEnumerable<Point3F> points...
        /// <summary>
        /// 投射垂直平面
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="x">法向量x分量</param>
        /// <param name="y">法向量y分量</param>
        /// <param name="z">法向量z分量</param>
        /// <returns>投射后点云</returns>
        public Point3F[] ProjectPerpendicularPlane(IEnumerable<Point3F> points, float x, float y, float z)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证
            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }
            #endregion

            IntPtr pointer = CommonNative.ProjectPerpendicularPlane(points_, points_.Length, x, y, z);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] projectedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return projectedPoints;
        }
        #endregion

        #region # 投射平行线集合 —— Point3F[] ProjectParallelLines(IEnumerable<Point3F> points...
        /// <summary>
        /// 投射平行线集合
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="x">平行线方向x分量</param>
        /// <param name="y">平行线方向y分量</param>
        /// <param name="z">平行线方向z分量</param>
        /// <returns>投射后点云</returns>
        /// <remarks>投射平行线集合</remarks>
        public Point3F[] ProjectParallelLines(IEnumerable<Point3F> points, float x, float y, float z)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = CommonNative.ProjectParallelLines(points_, points_.Length, x, y, z);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] projectedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return projectedPoints;
        }
        #endregion

        #region # 投射法向平面 —— Point3F[] ProjectNormalPlane(IEnumerable<Point3F> points...
        /// <summary>
        /// 投射法向平面
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="x">法向量x分量</param>
        /// <param name="y">法向量y分量</param>
        /// <param name="z">法向量z分量</param>
        /// <param name="nx">法向量x分量</param>
        /// <param name="ny">法向量y分量</param>
        /// <param name="nz">法向量z分量</param>
        /// <returns>投射后点云</returns>
        /// <remarks>投射法向平面</remarks>
        public Point3F[] ProjectNormalPlane(IEnumerable<Point3F> points, float x, float y, float z, float nx, float ny, float nz)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = CommonNative.ProjectNormalPlane(points_, points_.Length, x, y, z, nx, ny, nz);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] projectedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return projectedPoints;
        }
        #endregion

        #region # 投射法向球 —— Point3F[] ProjectNormalSphere(IEnumerable<Point3F> points...
        /// <summary>
        /// 投射法向球
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="x">球心x坐标</param>
        /// <param name="y">球心y坐标</param>
        /// <param name="z">球心z坐标</param>
        /// <param name="radius">半径</param>
        /// <param name="nx">法向量x分量</param>
        /// <param name="ny">法向量y分量</param>
        /// <param name="nz">法向量z分量</param>
        /// <returns>投射后点云</returns>
        /// <remarks>投射法向球</remarks>
        public Point3F[] ProjectNormalSphere(IEnumerable<Point3F> points, float x, float y, float z, float radius, float nx, float ny, float nz)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = CommonNative.ProjectNormalSphere(points_, points_.Length, x, y, z, radius, nx, ny, nz);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] projectedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return projectedPoints;
        }
        #endregion

        #region # 投射注册模型 —— Point3F[] ProjectRegistration(IEnumerable<Point3F> points...
        /// <summary>
        /// 投射注册模型
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="tx">平移向量x分量</param>
        /// <param name="ty">平移向量y分量</param>
        /// <param name="tz">平移向量z分量</param>
        /// <param name="qx">四元数x分量</param>
        /// <param name="qy">四元数y分量</param>
        /// <param name="qz">四元数z分量</param>
        /// <param name="qw">四元数w分量</param>
        /// <returns>投射后点云</returns>
        /// <remarks>投射注册模型</remarks>
        public Point3F[] ProjectRegistration(IEnumerable<Point3F> points, float tx, float ty, float tz, float qx, float qy, float qz, float qw)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = CommonNative.ProjectRegistration(points_, points_.Length, tx, ty, tz, qx, qy, qz, qw);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] projectedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return projectedPoints;
        }
        #endregion

        #region # 投射2D注册模型 —— Point3F[] ProjectRegistration2D(IEnumerable<Point3F> points...
        /// <summary>
        /// 投射2D注册模型
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="tx">平移向量x分量</param>
        /// <param name="ty">平移向量y分量</param>
        /// <param name="tz">平移向量z分量</param>
        /// <param name="qx">四元数x分量</param>
        /// <param name="qy">四元数y分量</param>
        /// <param name="qz">四元数z分量</param>
        /// <param name="qw">四元数w分量</param>
        /// <returns>投射后点云</returns>
        /// <remarks>投射2D注册模型</remarks>
        public Point3F[] ProjectRegistration2D(IEnumerable<Point3F> points, float tx, float ty, float tz, float qx, float qy, float qz, float qw)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = CommonNative.ProjectRegistration2D(points_, points_.Length, tx, ty, tz, qx, qy, qz, qw);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] projectedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return projectedPoints;
        }
        #endregion

        #region # 投射平行平面 —— Point3F[] ProjectParallelPlane(IEnumerable<Point3F> points...
        /// <summary>
        /// 投射平行平面
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="x">法向量x分量</param>
        /// <param name="y">法向量y分量</param>
        /// <param name="z">法向量z分量</param>
        /// <returns>投射后点云</returns>
        /// <remarks>投射平行平面</remarks>
        public Point3F[] ProjectParallelPlane(IEnumerable<Point3F> points, float x, float y, float z)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = CommonNative.ProjectParallelPlane(points_, points_.Length, x, y, z);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] projectedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return projectedPoints;
        }
        #endregion

        #region # 投射平行法向平面 —— Point3F[] ProjectNormalParallelPlane(IEnumerable<Point3F> points...
        /// <summary>
        /// 投射平行法向平面
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="x">法向量x分量</param>
        /// <param name="y">法向量y分量</param>
        /// <param name="z">法向量z分量</param>
        /// <returns>投射后点云</returns>
        /// <remarks>投射平行法向平面</remarks>
        public Point3F[] ProjectNormalParallelPlane(IEnumerable<Point3F> points, float x, float y, float z)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = CommonNative.ProjectNormalParallelPlane(points_, points_.Length, x, y, z);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] projectedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return projectedPoints;
        }
        #endregion

        #region # 投射棒状物 —— Point3F[] ProjectStick(IEnumerable<Point3F> points...
        /// <summary>
        /// 投射棒状物
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="x1">棒状物起点x坐标</param>
        /// <param name="y1">棒状物起点y坐标</param>
        /// <param name="z1">棒状物起点z坐标</param>
        /// <param name="x2">棒状物终点x坐标</param>
        /// <param name="y2">棒状物终点y坐标</param>
        /// <param name="z2">棒状物终点z坐标</param>
        /// <returns>投射后点云</returns>
        /// <remarks>投射棒状物</remarks>
        public Point3F[] ProjectStick(IEnumerable<Point3F> points, float x1, float y1, float z1, float x2, float y2, float z2)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = CommonNative.ProjectStick(points_, points_.Length, x1, y1, z1, x2, y2, z2);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] projectedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return projectedPoints;
        }
        #endregion

        #region # 投射3D椭圆 —— Point3F[] ProjectEllipse3D(IEnumerable<Point3F> points...
        /// <summary>
        /// 投射3D椭圆
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="x">椭圆中心x坐标</param>
        /// <param name="y">椭圆中心y坐标</param>
        /// <param name="z">椭圆中心z坐标</param>
        /// <param name="a">长轴半径</param>
        /// <param name="b">短轴半径</param>
        /// <param name="c">椭圆法向量x分量</param>
        /// <param name="d">椭圆法向量y分量</param>
        /// <param name="e">椭圆法向量z分量</param>
        /// <returns>投射后点云</returns>
        /// <remarks>投射3D椭圆</remarks>
        public Point3F[] ProjectEllipse3D(IEnumerable<Point3F> points, float x, float y, float z, float a, float b, float c, float d, float e)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = CommonNative.ProjectEllipse3D(points_, points_.Length, x, y, z, a, b, c, d, e);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] projectedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return projectedPoints;
        }
        #endregion

        #region # 提取边框 —— Point3F[] ExtractBorder(IEnumerable<Point3F> points)
        /// <summary>
        /// 提取边框
        /// </summary>
        /// <param name="points">点集</param>
        /// <returns>边框点云</returns>
        public Point3F[] ExtractBorder(IEnumerable<Point3F> points)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = CommonNative.ExtractBorder(points_, points_.Length);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] borderPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return borderPoints;
        }
        #endregion

        #region # 提取边界 —— Point3F[] ExtractBoundary(IEnumerable<Point3F> points...
        /// <summary>
        /// 提取边界
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="normalK">法向量K</param>
        /// <param name="featureRadius">特征半径</param>
        /// <param name="angleThreshold">角度阈值</param>
        /// <param name="threadsCount">线程数</param>
        /// <returns>边界点云</returns>
        public Point3F[] ExtractBoundary(IEnumerable<Point3F> points, int normalK, float featureRadius, float angleThreshold, int threadsCount)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = CommonNative.ExtractBoundary(points_, points_.Length, normalK, featureRadius, angleThreshold, threadsCount);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] boundaryPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return boundaryPoints;
        }
        #endregion
    }
}
