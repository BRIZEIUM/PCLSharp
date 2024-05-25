﻿using PCLSharp.Primitives.Models;
using System;
using System.Collections.Generic;
using System.Linq;

namespace PCLSharp.Primitives.Extensions
{
    /// <summary>
    /// 基元扩展
    /// </summary>
    public static unsafe class PrimitivesExtension
    {
        #region # 获取坐标点 —— static Point3F GetPoint(this Point3Normal3 pointNormal)
        /// <summary>
        /// 获取坐标点
        /// </summary>
        /// <param name="pointNormal">坐标点法向量</param>
        /// <returns>坐标点</returns>
        public static Point3F GetPoint(this Point3Normal3 pointNormal)
        {
            return new Point3F(pointNormal.X, pointNormal.Y, pointNormal.Z);
        }
        #endregion

        #region # 获取法向量 —— static Normal3F GetNormal(this Point3Normal3 pointNormal)
        /// <summary>
        /// 获取法向量
        /// </summary>
        /// <param name="pointNormal">坐标点法向量</param>
        /// <returns>法向量</returns>
        public static Normal3F GetNormal(this Point3Normal3 pointNormal)
        {
            return new Normal3F(pointNormal.NX, pointNormal.NY, pointNormal.NZ);
        }
        #endregion

        #region # 获取坐标点 —— static Point3F GetPoint(this Point3Color4 pointColor)
        /// <summary>
        /// 获取坐标点
        /// </summary>
        /// <param name="pointColor">坐标点颜色</param>
        /// <returns>坐标点</returns>
        public static Point3F GetPoint(this Point3Color4 pointColor)
        {
            return new Point3F(pointColor.X, pointColor.Y, pointColor.Z);
        }
        #endregion

        #region # 获取颜色 —— static Color4F GetColor(this Point3Color4 pointColor)
        /// <summary>
        /// 获取颜色
        /// </summary>
        /// <param name="pointColor">坐标点颜色</param>
        /// <returns>颜色</returns>
        public static Color4F GetColor(this Point3Color4 pointColor)
        {
            return new Color4F(pointColor.R, pointColor.G, pointColor.B, pointColor.A);
        }
        #endregion

        #region # 过滤NaN坐标点 —— static Point3F[] FilterNaN(this Point3F[] point3Fs)
        /// <summary>
        /// 过滤NaN坐标点
        /// </summary>
        public static Point3F[] FilterNaN(this Point3F[] point3Fs)
        {
            #region # 验证

            if (point3Fs == null)
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            Func<Point3F, bool> condition =
                point =>
                    !float.IsNaN(point.X) &&
                    !float.IsNaN(point.Y) &&
                    !float.IsNaN(point.Z);
            Point3F[] filteredPoint3Fs = point3Fs.Where(condition).ToArray();

            return filteredPoint3Fs;
        }
        #endregion

        #region # 过滤NaN坐标点 —— static Point3F[] FilterNaN(this IEnumerable<Point3F> point3Fs)
        /// <summary>
        /// 过滤NaN坐标点
        /// </summary>
        public static Point3F[] FilterNaN(this IEnumerable<Point3F> point3Fs)
        {
            #region # 验证

            if (point3Fs == null)
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            Func<Point3F, bool> condition =
                point =>
                    !float.IsNaN(point.X) &&
                    !float.IsNaN(point.Y) &&
                    !float.IsNaN(point.Z);
            Point3F[] filteredPoint3Fs = point3Fs.Where(condition).ToArray();

            return filteredPoint3Fs;
        }
        #endregion

        #region # 过滤NaN坐标点 —— static Point3Normal3[] FilterNaN(this Point3Normal3[] pointNormals)
        /// <summary>
        /// 过滤NaN坐标点
        /// </summary>
        public static Point3Normal3[] FilterNaN(this Point3Normal3[] pointNormals)
        {
            #region # 验证

            if (pointNormals == null)
            {
                return Array.Empty<Point3Normal3>();
            }

            #endregion

            Func<Point3Normal3, bool> condition =
                point =>
                    !float.IsNaN(point.X) &&
                    !float.IsNaN(point.Y) &&
                    !float.IsNaN(point.Z);
            Point3Normal3[] filteredPointNormals = pointNormals.Where(condition).ToArray();

            return filteredPointNormals;
        }
        #endregion

        #region # 过滤NaN坐标点 —— static Point3Normal3[] FilterNaN(this IEnumerable<Point3Normal3> pointNormals)
        /// <summary>
        /// 过滤NaN坐标点
        /// </summary>
        public static Point3Normal3[] FilterNaN(this IEnumerable<Point3Normal3> pointNormals)
        {
            #region # 验证

            if (pointNormals == null)
            {
                return Array.Empty<Point3Normal3>();
            }

            #endregion

            Func<Point3Normal3, bool> condition =
                point =>
                    !float.IsNaN(point.X) &&
                    !float.IsNaN(point.Y) &&
                    !float.IsNaN(point.Z);
            Point3Normal3[] filteredPointNormals = pointNormals.Where(condition).ToArray();

            return filteredPointNormals;
        }
        #endregion

        #region # 过滤NaN坐标点 —— static Point3Color4[] FilterNaN(this Point3Color4[] pointColors)
        /// <summary>
        /// 过滤NaN坐标点
        /// </summary>
        public static Point3Color4[] FilterNaN(this Point3Color4[] pointColors)
        {
            #region # 验证

            if (pointColors == null)
            {
                return Array.Empty<Point3Color4>();
            }

            #endregion

            Func<Point3Color4, bool> condition =
                point =>
                    !float.IsNaN(point.X) &&
                    !float.IsNaN(point.Y) &&
                    !float.IsNaN(point.Z);
            Point3Color4[] filteredPointColors = pointColors.Where(condition).ToArray();

            return filteredPointColors;
        }
        #endregion

        #region # 过滤NaN坐标点 —— static Point3Color4[] FilterNaN(this IEnumerable<Point3Color4> pointColors)
        /// <summary>
        /// 过滤NaN坐标点
        /// </summary>
        public static Point3Color4[] FilterNaN(this IEnumerable<Point3Color4> pointColors)
        {
            #region # 验证

            if (pointColors == null)
            {
                return Array.Empty<Point3Color4>();
            }

            #endregion

            Func<Point3Color4, bool> condition =
                point =>
                    !float.IsNaN(point.X) &&
                    !float.IsNaN(point.Y) &&
                    !float.IsNaN(point.Z);
            Point3Color4[] filteredPointColors = pointColors.Where(condition).ToArray();

            return filteredPointColors;
        }
        #endregion

        #region # 过滤NaN法向量 —— static Normal3F[] FilterNaN(this Normal3F[] normal3Fs)
        /// <summary>
        /// 过滤NaN法向量
        /// </summary>
        public static Normal3F[] FilterNaN(this Normal3F[] normal3Fs)
        {
            #region # 验证

            if (normal3Fs == null)
            {
                return Array.Empty<Normal3F>();
            }

            #endregion

            Func<Normal3F, bool> condition =
                normal =>
                    !float.IsNaN(normal.NX) &&
                    !float.IsNaN(normal.NY) &&
                    !float.IsNaN(normal.NZ);
            Normal3F[] filteredNormal3Fs = normal3Fs.Where(condition).ToArray();

            return filteredNormal3Fs;
        }
        #endregion

        #region # 过滤NaN法向量 —— static Normal3F[] FilterNaN(this IEnumerable<Normal3F> normal3Fs)
        /// <summary>
        /// 过滤NaN法向量
        /// </summary>
        public static Normal3F[] FilterNaN(this IEnumerable<Normal3F> normal3Fs)
        {
            #region # 验证

            if (normal3Fs == null)
            {
                return Array.Empty<Normal3F>();
            }

            #endregion

            Func<Normal3F, bool> condition =
                normal =>
                    !float.IsNaN(normal.NX) &&
                    !float.IsNaN(normal.NY) &&
                    !float.IsNaN(normal.NZ);
            Normal3F[] filteredNormal3Fs = normal3Fs.Where(condition).ToArray();

            return filteredNormal3Fs;
        }
        #endregion

        #region # 坐标点集结构体映射坐标点数组 —— static Point3F[] ToPoint3Fs(this Point3Fs point3Fs)
        /// <summary>
        /// 坐标点集结构体映射坐标点数组
        /// </summary>
        public static Point3F[] ToPoint3Fs(this Point3Fs point3Fs)
        {
            Span<Point3F> span = new Span<Point3F>(point3Fs.Points.ToPointer(), point3Fs.Length);
            Point3F[] points = span.ToArray();

            return points;
        }
        #endregion

        #region # 法向量集结构体映射法向量数组 —— static Normal3F[] ToNormal3Fs(this Normal3Fs normal3Fs)
        /// <summary>
        /// 法向量集结构体映射法向量数组
        /// </summary>
        public static Normal3F[] ToNormal3Fs(this Normal3Fs normal3Fs)
        {
            Span<Normal3F> span = new Span<Normal3F>(normal3Fs.Normals.ToPointer(), normal3Fs.Length);
            Normal3F[] normals = span.ToArray();

            return normals;
        }
        #endregion

        #region # 坐标点法向量集结构体映射坐标点法向量数组 —— static Point3Normal3[] ToPoint3Normal3s(this Point3Normal3s...
        /// <summary>
        /// 坐标点法向量集结构体映射坐标点法向量数组
        /// </summary>
        public static Point3Normal3[] ToPoint3Normal3s(this Point3Normal3s point3Normal3s)
        {
            Span<Point3Normal3> span = new Span<Point3Normal3>(point3Normal3s.PointNormals.ToPointer(), point3Normal3s.Length);
            Point3Normal3[] pointNormals = span.ToArray();

            return pointNormals;
        }
        #endregion

        #region # 坐标点颜色集结构体映射坐标点颜色数组 —— static Point3Color4[] ToPoint3Color4s(this Point3Color4s...
        /// <summary>
        /// 坐标点颜色集结构体映射坐标点颜色数组
        /// </summary>
        public static Point3Color4[] ToPoint3Color4s(this Point3Color4s point3Color4s)
        {
            Span<Point3Color4> span = new Span<Point3Color4>(point3Color4s.PointColors.ToPointer(), point3Color4s.Length);
            Point3Color4[] pointColors = span.ToArray();

            return pointColors;
        }
        #endregion
    }
}
