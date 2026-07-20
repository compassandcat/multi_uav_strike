// SPDX-License-Identifier: MIT
//
// One Euro Filter — 一维自适应低通滤波器
//   论文: Casiez et al. "1€ Filter: A Simple Speed-based Low-pass Filter for Noisy
//          Input in Interactive Systems", CHI 2012.
//   核心特性:
//     - 同时平滑值 + 平滑导数
//     - cutoff = min_cutoff + beta · |平滑速度|
//       → 静止/慢速时 cutoff 接近 min_cutoff,大幅平滑去抖
//       → 快速运动时 cutoff 抬升,几乎不滞后(跟得上)
//   适用:控制回路前的位置/速度/姿态抖动抑制(救援/机器人常见需求)
//
// 用法:
//   OneEuroFilter1D f;   // 默认参数(1.0Hz / 0.007 / 1.0Hz,适配慢速行人)
//   f.filter(x, dt);     // 第一帧自动 init,直接返回 x
//   f.reset();           // 切换物理目标 / 解锁时调用,丢掉 x_prev/dx_prev
//
#ifndef MULTI_UAV_STRIKE_ONE_EURO_FILTER_H
#define MULTI_UAV_STRIKE_ONE_EURO_FILTER_H

#include <cmath>
#include <algorithm>

namespace multi_uav_strike {

struct OneEuroFilter1D {
    // ---- 参数(可调) ----
    double min_cutoff;     // Hz,静止时最低截止频率(越小越平滑,但低速滞后越大)
    double beta;           // 速度系数(越大越跟得上快速运动,救援慢速目标建议 0.007)
    double d_cutoff;       // Hz,导数通道截止频率,通常固定 ~1.0Hz

    // ---- 状态(由 filter() 内部维护,reset() 清零) ----
    double x_prev;         // 上次滤波输出值
    double dx_prev;        // 上次滤波输出导数
    bool   initialized;    // 第一次 filter() 调用时直接 init,不滤波

    OneEuroFilter1D(double min_c = 1.0, double b = 0.007, double d_c = 1.0)
        : min_cutoff(min_c), beta(b), d_cutoff(d_c),
          x_prev(0.0), dx_prev(0.0), initialized(false) {}

    // 截止频率 → 平滑系数:α = 1 / (1 + te / tau),tau = 1/(2π·fc)
    static double smoothing_factor(double te, double cutoff_hz) {
        const double tau = 1.0 / (2.0 * M_PI * std::max(cutoff_hz, 1e-6));
        return 1.0 / (1.0 + te / tau);
    }

    // 主滤波接口:输入 x 与时间间隔 te(秒),返回平滑后的 x_hat
    double filter(double x, double te) {
        if (!initialized) {
            x_prev     = x;
            dx_prev    = 0.0;
            initialized = true;
            return x;
        }
        if (te <= 0.0) te = 1e-3;  // 防御:同帧多次调用 / dt=0 不会爆

        // 导数通道(平滑原始速度估计)
        const double dx_raw  = (x - x_prev) / te;
        const double alpha_d = smoothing_factor(te, d_cutoff);
        const double edx     = alpha_d * dx_raw + (1.0 - alpha_d) * dx_prev;

        // 值通道:cutoff 随平滑速度自适应
        const double cutoff  = min_cutoff + beta * std::fabs(edx);
        const double alpha   = smoothing_factor(te, cutoff);
        const double x_hat   = alpha * x + (1.0 - alpha) * x_prev;

        x_prev  = x_hat;
        dx_prev = edx;
        return x_hat;
    }

    // 丢弃内部状态(切换物理目标 / 解锁 / 跨长时间断流)
    void reset() {
        initialized = false;
        x_prev     = 0.0;
        dx_prev    = 0.0;
    }
};

}  // namespace multi_uav_strike

#endif  // MULTI_UAV_STRIKE_ONE_EURO_FILTER_H