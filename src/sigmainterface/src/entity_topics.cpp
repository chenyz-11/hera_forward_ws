#include "entity_topics.h"

#include "entity_naming.h"

EntityTopicManager::EntityTopicManager(rclcpp::Node& node, const std::string& root)
    : node_(node), root_(root)
{
}

void EntityTopicManager::publishModelInfo(int64_t raw, const ModelInfo& msg)
{
    std::lock_guard<std::mutex> lk(res_mutex_);
    const auto it = model_pubs_.find(raw);
    if (it != model_pubs_.end())
        it->second->publish(msg);   // publish 线程安全，可持锁直发
    else
        pending_model_[raw].push_back(msg);   // 首条入队，维护定时器开通后补发
}

void EntityTopicManager::publishFusion(int64_t raw, const TargetFusion& msg)
{
    std::lock_guard<std::mutex> lk(res_mutex_);
    const auto it = fusion_pubs_.find(raw);
    if (it != fusion_pubs_.end())
        it->second->publish(msg);
    else
        pending_fusion_[raw].push_back(msg);
}

void EntityTopicManager::publishWeaponFireAck(int64_t raw, const WeaponFireAck& msg)
{
    std::lock_guard<std::mutex> lk(res_mutex_);
    const auto it = ack_pubs_.find(raw);
    if (it != ack_pubs_.end())
        it->second->publish(msg);
    else
        pending_ack_[raw].push_back(msg);
}

void EntityTopicManager::maintain()
{
    std::lock_guard<std::mutex> lk(res_mutex_);

    for (auto& [raw, msgs] : pending_model_)
    {
        SigmaModelType type; uint8_t camp, squad; uint32_t model;
        parseEntityID(raw, type, camp, squad, model);
        const std::string ns = entity_naming::entityNs(root_, type, camp, squad, model);
        auto& pub = model_pubs_[raw];
        if (!pub)
        {
            pub = node_.create_publisher<ModelInfo>(ns + "/model_info", 10);
            RCLCPP_INFO(node_.get_logger(), "实体话题已开通: %s (model_info)", ns.c_str());
        }
        for (const auto& m : msgs)
            pub->publish(m);

        const auto opened = ensureCommandSubsLocked(raw);
        if (!opened.empty())
        {
            std::string joined;
            for (const auto& t : opened)
                joined += (joined.empty() ? "" : ", ") + t;
            RCLCPP_INFO(node_.get_logger(), "实体指挥话题已开通: %s", joined.c_str());
        }
    }
    pending_model_.clear();

    for (auto& [raw, msgs] : pending_fusion_)
    {
        SigmaModelType type; uint8_t camp, squad; uint32_t model;
        parseEntityID(raw, type, camp, squad, model);
        const std::string ns = entity_naming::entityNs(root_, type, camp, squad, model);
        auto& pub = fusion_pubs_[raw];
        if (!pub)
        {
            pub = node_.create_publisher<TargetFusion>(ns + "/target_fusion", 10);
            RCLCPP_INFO(node_.get_logger(), "实体话题已开通: %s (target_fusion)", ns.c_str());
        }
        for (const auto& m : msgs)
            pub->publish(m);
    }
    pending_fusion_.clear();

    for (auto& [raw, msgs] : pending_ack_)
    {
        SigmaModelType type; uint8_t camp, squad; uint32_t model;
        parseEntityID(raw, type, camp, squad, model);
        const std::string ns = entity_naming::entityNs(root_, type, camp, squad, model);
        auto& pub = ack_pubs_[raw];
        if (!pub)
        {
            pub = node_.create_publisher<WeaponFireAck>(ns + "/weapon_fire_ack", 10);
            RCLCPP_INFO(node_.get_logger(), "实体话题已开通: %s (weapon_fire_ack)", ns.c_str());
        }
        for (const auto& m : msgs)
            pub->publish(m);
    }
    pending_ack_.clear();
}

std::vector<std::string> EntityTopicManager::preopenCommands(int64_t raw)
{
    std::lock_guard<std::mutex> lk(res_mutex_);
    return ensureCommandSubsLocked(raw);
}

std::vector<std::string> EntityTopicManager::ensureCommandSubsLocked(int64_t raw)
{
    std::vector<std::string> created;

    SigmaModelType type; uint8_t camp, squad; uint32_t model;
    parseEntityID(raw, type, camp, squad, model);
    const std::string ns = entity_naming::entityNs(root_, type, camp, squad, model);

    auto& subs = command_subs_[raw];
    for (const auto& spec : command_specs_)
    {
        if (subs.count(spec.suffix))
            continue;
        const std::string topic = ns + "/" + spec.suffix;
        subs[spec.suffix] = spec.attach(node_, topic, raw);   // shared_ptr<void> 保活订阅
        created.push_back(topic);
    }
    return created;
}
