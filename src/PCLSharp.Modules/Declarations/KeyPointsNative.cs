﻿using PCLSharp.Primitives.Constants;
using PCLSharp.Primitives.Models;
using System;
using System.Runtime.InteropServices;

namespace PCLSharp.Modules.Declarations
{
    /// <summary>
    /// 点云关键点声明
    /// </summary>
    public static class KeyPointsNative
    {
        #region # 计算NARF关键点 —— static extern IntPtr ComputeNARF(Point3F[] points...
        /// <summary>
        /// 计算NARF关键点
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="angularResolution">角度分辨率</param>
        /// <param name="maxAngleWidth">传感器水平边界角度</param>
        /// <param name="maxAngleHeight">传感器垂直边界角度</param>
        /// <param name="noiseLevel">所有与最近点的最大距离</param>
        /// <param name="minRange">最小可见范围</param>
        /// <param name="borderSize">边界尺寸</param>
        /// <param name="supportSize">计算范围半径</param>
        /// <returns>NARF关键点集</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "computeNARF")]
        public static extern IntPtr ComputeNARF(Point3F[] points, int length, float angularResolution, float maxAngleWidth, float maxAngleHeight, float noiseLevel, float minRange, int borderSize, float supportSize);
        #endregion

        #region # 计算ISS关键点 —— static extern IntPtr ComputeISS(Point3F[] points...
        /// <summary>
        /// 计算ISS关键点
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="salientRadius">显著半径</param>
        /// <param name="nonMaxRadius">非极大值抑制半径</param>
        /// <param name="threshold21">二一特征值比上限</param>
        /// <param name="threshold32">三二特征值比上限</param>
        /// <param name="minNeighborsCount">最小邻域点数</param>
        /// <param name="threadsCount">线程数</param>
        /// <returns>ISS关键点集</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "computeISS")]
        public static extern IntPtr ComputeISS(Point3F[] points, int length, float salientRadius, float nonMaxRadius, float threshold21, float threshold32, int minNeighborsCount, int threadsCount);
        #endregion

        #region # 计算SIFT关键点 —— static extern IntPtr ComputeSIFT(Point3F[] points...
        /// <summary>
        /// 计算SIFT关键点
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="minScale">尺度空间最小标准偏差</param>
        /// <param name="octavesCount">金字塔组数量</param>
        /// <param name="scalesPerOctaveCount">每组金字塔计算尺度</param>
        /// <param name="minContrast">限制关键点检测阈值</param>
        /// <returns>SIFT关键点集</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "computeSIFT")]
        public static extern IntPtr ComputeSIFT(Point3F[] points, int length, float minScale, int octavesCount, int scalesPerOctaveCount, float minContrast);
        #endregion

        #region # 计算Harris关键点 —— static extern IntPtr ComputeHarris(Point3F[] points...
        /// <summary>
        /// 计算Harris关键点
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="nonMaxSupression">非极大值抑制</param>
        /// <param name="radius">搜索半径</param>
        /// <param name="threshold">感兴趣阈值</param>
        /// <returns>Harris关键点集</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "computeHarris")]
        public static extern IntPtr ComputeHarris(Point3F[] points, int length, bool nonMaxSupression, float radius, float threshold);
        #endregion

        #region # 计算SUSAN关键点 —— static extern IntPtr ComputeSUSAN(Point3F[] points...
        /// <summary>
        /// 计算SUSAN关键点
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="nonMaxSupression">非极大值抑制</param>
        /// <param name="radius">搜索半径</param>
        /// <param name="distanceThreshold">距离阈值</param>
        /// <param name="angularThreshold">角度阈值</param>
        /// <param name="intensityThreshold">强度阈值</param>
        /// <returns>SUSAN关键点集</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "computeSUSAN")]
        public static extern IntPtr ComputeSUSAN(Point3F[] points, int length, bool nonMaxSupression, float radius, float distanceThreshold, float angularThreshold, float intensityThreshold);
        #endregion
    }
}