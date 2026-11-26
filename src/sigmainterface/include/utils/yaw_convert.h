#ifndef SIGMA_INTERFACE_YAW_CONVERT_H
#define SIGMA_INTERFACE_YAW_CONVERT_H

/**
 * @file  yaw_convert.h
 * @brief 偏航角坐标系换算（UE 左手系度 <-> 项目 ENU 弧度）
 *
 * 本头为 sigma_interface 内部工具（src/），header-only 不对外暴露。
 * 约定：内部一律「ENU 弧度」——正东为 0、逆时针（向北偏）为正，规整到 [0, 2π)；
 * UE 侧偏航为「度」——正东为 0、顺时针（向南偏）为正，范围 [0, 360)，270° 为正北。
 * 两侧互为镜像：换算关系 θ_enu = −ψ_ue。
 * 只在协议边界（Receiver 接收 UE度->ENU弧度 / Transmitter 发送 ENU弧度->UE度）换算。
 */

#include "geo_convert.h"
#include <cmath>

namespace yaw_conv {

/** @brief UE 偏航（度，正东 0、顺时针为正）→ 项目 ENU 偏航（弧度，正东 0、逆时针为正，规整到 [0, 2π)） */
inline double ueDegToEnuRad(double ueDeg)
{
    double y = std::fmod(-ueDeg * M_PI / 180.0, 2.0 * M_PI);
    if (y < 0.0) y += 2.0 * M_PI;
    return y;
}

/** @brief 项目 ENU 偏航（弧度，正东 0、逆时针为正）→ UE 偏航（度，正东 0、顺时针为正，规整到 [0, 360)） */
inline double enuRadToUeDeg(double enuRad)
{
    double y = std::fmod(-enuRad, 2.0 * M_PI);
    if (y < 0.0) y += 2.0 * M_PI;
    return y * 180.0 / M_PI;
}

} // namespace yaw_conv

#endif // SIMCORE_YAW_CONVERT_H
