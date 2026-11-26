#ifndef SIGMA_INTERFACE_HOME_STATE_H
#define SIGMA_INTERFACE_HOME_STATE_H

/**
 * @file  home_state.h
 * @brief home 参考系状态机（仅 ROS 模式使用）
 *
 * home 来源两模式（由节点装配，本组件只管状态与等待）：
 *   - UE 的 ORIGIN 广播（SDK 解析线程回调写入）
 *   - ROS 参数 home_lat_deg / home_lon_deg / home_alt_m（节点构造期直接写入）
 * home 未就绪期间：调用方跳过下行换算、丢弃上行指令。
 * 线程安全：单写多读（写 = set，读 = get / ready / waitReady）。
 */

#include <atomic>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

class HomeState
{
public:
    HomeState() = default;

    /** @brief 写入 home 坐标（度/米），并置就绪 */
    void set(double latDeg, double lonDeg, double altM);

    /** @brief 取 home 坐标（度/米）；未就绪返回 false */
    bool get(double* latDeg, double* lonDeg, double* altM) const;

    /** @brief home 是否已就绪 */
    bool ready() const { return ready_.load(std::memory_order_acquire); }

    /**
     * @brief 限时等待 home 就绪（等 UE 的 ORIGIN 广播写入）
     * @param timeoutMs 等待毫秒数（须 > 0），超时抛异常；Ctrl-C（rclcpp::ok 变 false）随时退出
     */
    void waitReady(int timeoutMs, rclcpp::Logger logger) const;

private:
    struct Home { double latDeg, lonDeg, altM; };

    mutable std::mutex mutex_;
    Home home_{};
    std::atomic<bool> ready_{false};
};

#endif // SIGMA_INTERFACE_HOME_STATE_H
