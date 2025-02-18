﻿using HelixToolkit.Wpf.SharpDX;
using PCLSharp.Client.ViewModels.CommonContext;
using PCLSharp.Extensions.Helix;
using PCLSharp.Modules.Interfaces;
using PCLSharp.Primitives.Extensions;
using PCLSharp.Primitives.Models;
using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Extensions;
using SharpDX;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using Color = System.Windows.Media.Color;

namespace PCLSharp.Client.ViewModels.SegmentationContext
{
    /// <summary>
    /// 区域生长颜色分割视图模型
    /// </summary>
    public class RegionGrowColorViewModel : PreviewViewModel
    {
        #region # 字段及构造器

        /// <summary>
        /// 点云分割接口
        /// </summary>
        private readonly ICloudSegmentations _cloudSegmentations;

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public RegionGrowColorViewModel(ICloudCommon cloudCommon, ICloudSegmentations cloudSegmentations)
            : base(cloudCommon)
        {
            this._cloudSegmentations = cloudSegmentations;
        }

        #endregion

        #region # 属性

        #region 法向量K —— int? NormalK
        /// <summary>
        /// 法向量K
        /// </summary>
        [DependencyProperty]
        public int? NormalK { get; set; }
        #endregion

        #region 簇K —— int? ClusterK
        /// <summary>
        /// 簇K
        /// </summary>
        [DependencyProperty]
        public int? ClusterK { get; set; }
        #endregion

        #region 距离阈值 —— float? DistanceThreshold
        /// <summary>
        /// 距离阈值
        /// </summary>
        [DependencyProperty]
        public float? DistanceThreshold { get; set; }
        #endregion

        #region 平滑阈值（角度） —— float? SmoothnessThreshold
        /// <summary>
        /// 平滑阈值（角度）
        /// </summary>
        [DependencyProperty]
        public float? SmoothnessThreshold { get; set; }
        #endregion

        #region 曲率阈值 —— float? CurvatureThreshold
        /// <summary>
        /// 曲率阈值
        /// </summary>
        [DependencyProperty]
        public float? CurvatureThreshold { get; set; }
        #endregion

        #region 点颜色阈值 —— float? PointColorThreshold
        /// <summary>
        /// 点颜色阈值
        /// </summary>
        [DependencyProperty]
        public float? PointColorThreshold { get; set; }
        #endregion

        #region 区域颜色阈值 —— float? RegionColorThreshold
        /// <summary>
        /// 区域颜色阈值
        /// </summary>
        [DependencyProperty]
        public float? RegionColorThreshold { get; set; }
        #endregion

        #region 簇最小尺寸 —— int? MinClusterSize
        /// <summary>
        /// 簇最小尺寸
        /// </summary>
        [DependencyProperty]
        public int? MinClusterSize { get; set; }
        #endregion

        #region 簇最大尺寸 —— int? MaxClusterSize
        /// <summary>
        /// 簇最大尺寸
        /// </summary>
        [DependencyProperty]
        public int? MaxClusterSize { get; set; }
        #endregion

        #region 线程数 —— int? ThreadsCount
        /// <summary>
        /// 线程数
        /// </summary>
        [DependencyProperty]
        public int? ThreadsCount { get; set; }
        #endregion

        #endregion

        #region # 方法

        #region 初始化 —— override Task OnInitializeAsync(CancellationToken cancellationToken)
        /// <summary>
        /// 初始化
        /// </summary>
        protected override Task OnInitializeAsync(CancellationToken cancellationToken)
        {
            //默认值
            this.NormalK = 50;
            this.ClusterK = 30;
            this.DistanceThreshold = 10.0f;
            this.SmoothnessThreshold = 30.0f;
            this.CurvatureThreshold = 1.5f;
            this.PointColorThreshold = 6.0f;
            this.RegionColorThreshold = 5.0f;
            this.MinClusterSize = 1000;
            this.MaxClusterSize = 100000;
            this.ThreadsCount = 20;

            return base.OnInitializeAsync(cancellationToken);
        }
        #endregion

        #region 应用 —— async void Apply()
        /// <summary>
        /// 应用
        /// </summary>
        public async void Apply()
        {
            #region # 验证

            if (!this.NormalK.HasValue)
            {
                MessageBox.Show("法向量K不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.ClusterK.HasValue)
            {
                MessageBox.Show("簇K不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.DistanceThreshold.HasValue)
            {
                MessageBox.Show("距离阈值不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.SmoothnessThreshold.HasValue)
            {
                MessageBox.Show("平滑阈值不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.CurvatureThreshold.HasValue)
            {
                MessageBox.Show("曲率阈值不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.PointColorThreshold.HasValue)
            {
                MessageBox.Show("点颜色阈值不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.RegionColorThreshold.HasValue)
            {
                MessageBox.Show("区域颜色阈值不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.MinClusterSize.HasValue)
            {
                MessageBox.Show("簇最小尺寸不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.MaxClusterSize.HasValue)
            {
                MessageBox.Show("簇最大尺寸不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.ThreadsCount.HasValue)
            {
                MessageBox.Show("线程数不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (this.PointCloud == null)
            {
                MessageBox.Show("点云不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            Point3Color4[] pointColors = this.PointCloud.ToPoint3Color4s();
            Point3Color4[][] pointsClusters = await Task.Run(() => this._cloudSegmentations.RegionGrowingColorSegment(pointColors, this.NormalK!.Value, this.ClusterK!.Value, this.DistanceThreshold!.Value, this.SmoothnessThreshold!.Value, this.CurvatureThreshold!.Value, this.PointColorThreshold!.Value, this.RegionColorThreshold!.Value, this.MinClusterSize!.Value, this.MaxClusterSize!.Value, this.ThreadsCount!.Value));

            Vector3Collection positions = new Vector3Collection();
            Color4Collection colors = new Color4Collection();
            for (int clusterIndex = 0; clusterIndex < pointsClusters.Length; clusterIndex++)
            {
                Point3Color4[] pointsCluster = pointsClusters[clusterIndex];
                Color color = ColorExtension.RandomColor();
                if (clusterIndex % 2 == 0)
                {
                    color = color.Invert();
                }
                IEnumerable<Vector3> clusterPositions = pointsCluster.Select(x => x.GetPoint()).ToVector3s();
                IEnumerable<Color4> clusterColors = pointsCluster.Select(x => color.ToColor4());
                positions.AddRange(clusterPositions);
                colors.AddRange(clusterColors);
            }
            this.PointCloud = new PointGeometry3D
            {
                Positions = positions,
                Colors = colors
            };

            this.Idle();
        }
        #endregion

        #endregion
    }
}
