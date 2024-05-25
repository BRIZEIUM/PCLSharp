﻿using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
using System;
using System.Windows;

namespace PCLSharp.Client.ViewModels.FilterContext
{
    /// <summary>
    /// 随机采样视图模型
    /// </summary>
    public class RandomSampleViewModel : ScreenBase
    {
        #region # 字段及构造器

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public RandomSampleViewModel()
        {
            //默认值
            this.Seed = (int)Math.Abs(int.MaxValue - DateTime.Now.Ticks);
            this.SamplesCount = 2000;
        }

        #endregion

        #region # 属性

        #region 随机种子 —— int? Seed
        /// <summary>
        /// 随机种子
        /// </summary>
        [DependencyProperty]
        public int? Seed { get; set; }
        #endregion

        #region 采样数量 —— int? SamplesCount
        /// <summary>
        /// 采样数量
        /// </summary>
        [DependencyProperty]
        public int? SamplesCount { get; set; }
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

            if (!this.Seed.HasValue)
            {
                MessageBox.Show("随机种子不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.SamplesCount.HasValue)
            {
                MessageBox.Show("采样数量不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            await base.TryCloseAsync(true);
        }
        #endregion

        #endregion
    }
}
