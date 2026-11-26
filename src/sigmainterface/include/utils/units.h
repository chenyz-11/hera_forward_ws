#ifndef SIGMA_INTERFACE_UNITS_H
#define SIGMA_INTERFACE_UNITS_H

/**
 * @file  units.h
 * @brief 协议长度单位换算常量（UE 仿真世界使用厘米，业务侧统一使用米）
 *
 * 本头为 sigma_interface 内部工具（src/），header-only 不对外暴露。
 * 约定：内部一律「米」；仅在协议边界（Receiver 接收 cm->m /
 * Transmitter 发送 m->cm）做换算。
 * 经纬度整型（项目惯称「微度」，实际比例 1e-7 °/lsb，同 dxfd 协议）是
 * 角度量，换算常量见下方 kMicroDegToDeg / kDegToMicroDeg。
 */

namespace units {

/** @brief 厘米 -> 米 */
constexpr double kCmToM = 0.01;

/** @brief 米 -> 厘米 */
constexpr double kMToCm = 100.0;

/** @brief 协议经纬度整型（微度，1e-7 °/lsb） -> 度 */
constexpr double kMicroDegToDeg = 1e-7;

/** @brief 度 -> 协议经纬度整型（微度，1e-7 °/lsb） */
constexpr double kDegToMicroDeg = 1e7;

} // namespace units

#endif // SIGMA_INTERFACE_UNITS_H
