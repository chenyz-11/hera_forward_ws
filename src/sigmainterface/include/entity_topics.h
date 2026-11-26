#ifndef SIGMA_INTERFACE_ENTITY_TOPICS_H
#define SIGMA_INTERFACE_ENTITY_TOPICS_H

/**
 * @file  entity_topics.h
 * @brief 实体话题资源管理器：话题树目录 + pub/sub 生命周期 + 惰性开通（仅 ROS 模式使用）
 *
 * 话题树（root 为顶层 namespace，如 /sigma）：
 *   {root}/camp_{camp}/squad_{squad}/{型号名}_{model_id}/model_info       发布（该实体自身状态）
 *   {root}/camp_{camp}/squad_{squad}/{型号名}_{model_id}/target_fusion    发布（按探测方 self 归属）
 *   {root}/camp_{camp}/squad_{squad}/{型号名}_{model_id}/weapon_fire_ack  发布（按下发时记录的 seq 归属）
 *   {root}/camp_{camp}/squad_{squad}/{型号名}_{model_id}/{suffix}         订阅（registerCommand 注册）
 *   现有指令话题 suffix：motion_command / velocity_command /
 *                       reference_trajectory / weapon_fire_command
 *
 * 惰性开通线程模型（rclcpp 实体创建与 spin 中的 executor 并发不安全）：
 *   - SDK 解析线程：publishModelInfo / publishFusion 只查表直发；实体未开通则消息入 pending 队列
 *   - spin 线程：maintain()（维护定时器回调）/ preopenCommands()（启动期）统一创建 pub/sub 并按序补发积压
 *   - 实体首条消息最多延迟一个维护周期（kMaintainPeriodMs），其后零延迟
 *
 * 指令订阅的注册：registerCommand<MsgT> 在启动期（任何实体开通前）调用，
 * 内部按 suffix 记录一份“创建订阅”的工厂；实体开通时统一套用，新增指令话题
 * 无需再改本类的接口、成员表或 ensure 函数。
 *
 * 兼容模式复用说明：本组件只依赖 Node 引用与已解码数据，不感知 SDK 类型，
 * 可被未来 COMPAT 模式（core + ROS 双出口）直接实例化。
 */

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "sigma2ros_bridge/msg/model_info.hpp"
#include "sigma2ros_bridge/msg/target_fusion.hpp"
#include "sigma2ros_bridge/msg/weapon_fire_ack.hpp"

/** @brief 实体话题维护周期（ms）：pending 队列到 publisher/subscription 的开通延迟上限 */
constexpr int32_t kMaintainPeriodMs = 20;

class EntityTopicManager
{
public:
    /**
     * @param node 宿主节点（用于创建 pub/sub 与日志）
     * @param root 话题树顶层（如 "/sigma"，空串 = 无顶层命名空间）
     */
    EntityTopicManager(rclcpp::Node& node, const std::string& root);

    /**
     * @brief 注册一类实体指挥订阅（启动期、实体开通前调用）
     * @tparam MsgT 订阅的消息类型
     * @param suffix 相对实体目录的话题名，如 "motion_command"
     * @param cb     回调（spin 线程上下文）：raw = 话题路径对应实体，msg 为订阅消息
     */
    template <typename MsgT, typename Fn>
    void registerCommand(const std::string& suffix, Fn&& cb)
    {
        std::function<void(int64_t, const MsgT&)> handler(std::forward<Fn>(cb));
        std::lock_guard<std::mutex> lk(res_mutex_);
        command_specs_.push_back(CommandSpec{
            suffix,
            [handler](rclcpp::Node& node, const std::string& topic, int64_t raw) {
                return std::static_pointer_cast<void>(
                    node.create_subscription<MsgT>(
                        topic, 10,
                        [handler, raw](typename MsgT::SharedPtr msg) { handler(raw, *msg); }));
            }});
    }

    /** @brief SDK 解析线程：model_info 查表直发，实体未开通则入 pending 队列（publish 线程安全） */
    void publishModelInfo(int64_t raw, const sigma2ros_bridge::msg::ModelInfo& msg);

    /** @brief SDK 解析线程：target_fusion 按探测方 self 归属发布（publish 线程安全） */
    void publishFusion(int64_t raw, const sigma2ros_bridge::msg::TargetFusion& msg);

    /** @brief SDK 解析线程：weapon_fire_ack 按开火平台实体发布（publish 线程安全） */
    void publishWeaponFireAck(int64_t raw, const sigma2ros_bridge::msg::WeaponFireAck& msg);

    /** @brief spin 线程：维护定时器回调，pending 队列 -> 创建实体 pub/sub 并按序补发积压消息 */
    void maintain();

    /** @brief spin 线程（启动期）：预开通实体的全部已注册指令话题；返回本次新建的话题路径 */
    std::vector<std::string> preopenCommands(int64_t raw);

private:
    /** @brief 一类指令订阅：suffix + “按 (topic, raw) 创建订阅”的工厂（类型擦除保活） */
    struct CommandSpec
    {
        std::string suffix;
        std::function<std::shared_ptr<void>(rclcpp::Node&, const std::string&, int64_t)> attach;
    };

    /** @brief 确保实体的全部已注册指令订阅存在（res_mutex_ 持有态调用；仅 spin 线程）；返回新建话题路径 */
    std::vector<std::string> ensureCommandSubsLocked(int64_t raw);

    using ModelInfo    = sigma2ros_bridge::msg::ModelInfo;
    using TargetFusion = sigma2ros_bridge::msg::TargetFusion;
    using WeaponFireAck = sigma2ros_bridge::msg::WeaponFireAck;

    rclcpp::Node& node_;
    std::string root_;
    std::vector<CommandSpec> command_specs_;

    // 资源表：SDK 回调线程只读查表 + 入队；创建仅发生在 spin 线程
    std::mutex res_mutex_;
    std::unordered_map<int64_t, rclcpp::Publisher<ModelInfo>::SharedPtr> model_pubs_;
    std::unordered_map<int64_t, rclcpp::Publisher<TargetFusion>::SharedPtr> fusion_pubs_;
    std::unordered_map<int64_t, rclcpp::Publisher<WeaponFireAck>::SharedPtr> ack_pubs_;
    std::unordered_map<int64_t, std::unordered_map<std::string, std::shared_ptr<void>>> command_subs_;
    std::unordered_map<int64_t, std::vector<ModelInfo>> pending_model_;
    std::unordered_map<int64_t, std::vector<TargetFusion>> pending_fusion_;
    std::unordered_map<int64_t, std::vector<WeaponFireAck>> pending_ack_;
};

#endif // SIGMA_INTERFACE_ENTITY_TOPICS_H
