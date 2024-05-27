﻿using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
using System.Windows;

namespace PCLSharp.Client.ViewModels.FeatureContext
{
    /// <summary>
    /// SHOT特征视图模型
    /// </summary>
    public class ShotViewModel : ScreenBase
    {
        #region # 字段及构造器

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public ShotViewModel()
        {
            //默认值
            this.NormalK = 5;
            this.FeatureRadius = 0.4f;
            this.ThreadsCount = 20;
            this.ImageWidth = 1920;
            this.ImageHeight = 1080;
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

        #region 特征搜索半径 —— float? FeatureRadius
        /// <summary>
        /// 特征搜索半径
        /// </summary>
        [DependencyProperty]
        public float? FeatureRadius { get; set; }
        #endregion

        #region 线程数 —— int? ThreadsCount
        /// <summary>
        /// 线程数
        /// </summary>
        [DependencyProperty]
        public int? ThreadsCount { get; set; }
        #endregion

        #region 直方图宽度 —— int? ImageWidth
        /// <summary>
        /// 直方图宽度
        /// </summary>
        [DependencyProperty]
        public int? ImageWidth { get; set; }
        #endregion

        #region 直方图高度 —— int? ImageHeight
        /// <summary>
        /// 直方图高度
        /// </summary>
        [DependencyProperty]
        public int? ImageHeight { get; set; }
        #endregion

        #endregion

        #region # 方法

        #region 提交 —— async void Submit()
        /// <summary>
        /// 提交
        /// </summary>
        public async void Submit()
        {
            #region # 验证

            if (!this.NormalK.HasValue)
            {
                MessageBox.Show("法向量K不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.FeatureRadius.HasValue)
            {
                MessageBox.Show("特征搜索半径不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.ThreadsCount.HasValue)
            {
                MessageBox.Show("线程数不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.ImageWidth.HasValue)
            {
                MessageBox.Show("直方图宽度不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.ImageHeight.HasValue)
            {
                MessageBox.Show("直方图高度不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            await base.TryCloseAsync(true);
        }
        #endregion

        #endregion
    }
}
