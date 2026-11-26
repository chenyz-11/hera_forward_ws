#include "home_state.h"

#include <chrono>
#include <stdexcept>
#include <thread>

void HomeState::set(double latDeg, double lonDeg, double altM)
{
    {
        std::lock_guard<std::mutex> lk(mutex_);
        home_ = {latDeg, lonDeg, altM};
    }
    ready_.store(true, std::memory_order_release);
}

bool HomeState::get(double* latDeg, double* lonDeg, double* altM) const
{
    if (!ready_.load(std::memory_order_acquire))
        return false;
    std::lock_guard<std::mutex> lk(mutex_);
    *latDeg = home_.latDeg;
    *lonDeg = home_.lonDeg;
    *altM   = home_.altM;
    return true;
}

void HomeState::waitReady(int timeoutMs, rclcpp::Logger logger) const
{
    using Clock = std::chrono::steady_clock;
    const auto deadline = Clock::now() + std::chrono::milliseconds(timeoutMs);

    while (!ready_.load(std::memory_order_acquire))
    {
        if (!rclcpp::ok())
            throw std::runtime_error("等待 home 期间收到关闭信号（Ctrl-C），退出");
        if (Clock::now() >= deadline)
        {
            RCLCPP_FATAL(logger,
                "home_wait_ms=%d 内未收到 UE 的 ORIGIN 广播；UE 广播不可达时请设置 home_source:=config 用参数指定 home",
                timeoutMs);
            throw std::runtime_error("等待 ORIGIN 广播超时");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
