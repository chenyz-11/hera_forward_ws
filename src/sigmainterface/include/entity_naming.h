#ifndef SIGMA_INTERFACE_ENTITY_NAMING_H
#define SIGMA_INTERFACE_ENTITY_NAMING_H

/**
 * @file  entity_naming.h
 * @brief 实体标识与话题命名工具（header-only，仅 ROS 模式使用）
 *
 * 职责：
 *   - 协议 64 位 entityID <-> EntityId msg 解码分量互转（fillEntityId / entityToRaw）
 *   - 话题树实体目录命名（entityNs）：{root}/camp_{camp}/squad_{squad}/{型号名}_{model_id}
 *   - pull_contents 实体串 "camp_squad_型号_model" 解析（parseEntityString / EntityString）
 * 跨翻译单元共享的自由函数一律 inline，避免重复定义链接错误。
 */

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include "sigmaAssist.h"   // getEntityID/parseEntityID：协议 64 位 entityID 编解码

// rosidl 生成的消息头（包名小写）
#include "sigma2ros_bridge/msg/entity_id.hpp"

namespace entity_naming {

/** @brief 协议 64 位 entityID -> msg 解码分量（parseEntityID 的 msg 填充版） */
inline void fillEntityId(sigma2ros_bridge::msg::EntityId& out, int64_t raw)
{
    SigmaModelType type;
    uint8_t camp, squad;
    uint32_t model;
    parseEntityID(raw, type, camp, squad, model);
    out.model_type = static_cast<uint16_t>(type);
    out.camp_id    = camp;
    out.squad_id   = squad;
    out.model_id   = model;
}

/** @brief msg 解码分量 -> 协议 64 位 entityID（getEntityID 的 msg 读取版） */
inline int64_t entityToRaw(const sigma2ros_bridge::msg::EntityId& id)
{
    return getEntityID(static_cast<SigmaModelType>(id.model_type),
                       id.camp_id, id.squad_id, id.model_id);
}

/** @brief 型号名的 topic 安全形式：AssetTypeToString 结果中的 '-' 换 '_'（ROS2 话题名不允许 '-'），未知枚举回退 type_{n} */
inline std::string modelTypeName(SigmaModelType type)
{
    try
    {
        std::string name = AssetTypeToString(type);
        std::replace(name.begin(), name.end(), '-', '_');
        return name;
    }
    catch (...)
    {
        return "type_" + std::to_string(static_cast<unsigned>(type));
    }
}

/** @brief 型号名 -> 枚举：先试原串，失败再试 '_' 换 '-'（兼容 sigmaAssist 的 "AA-Patriot" 连字符约定） */
inline SigmaModelType tryStringToAssetType(const std::string& name)
{
    try
    {
        return StringToAssetType(name);
    }
    catch (...)
    {
        std::string alt = name;
        std::replace(alt.begin(), alt.end(), '_', '-');
        return StringToAssetType(alt);   // 仍失败则异常上抛，由调用方 catch
    }
}

/** @brief 实体话题目录：{root}/camp_{camp}/squad_{squad}/{型号名}_{model_id} */
inline std::string entityNs(const std::string& root, SigmaModelType type,
                            uint8_t camp, uint8_t squad, uint32_t model)
{
    return root + "/camp_" + std::to_string(camp)
         + "/squad_" + std::to_string(squad)
         + "/" + modelTypeName(type) + "_" + std::to_string(model);
}

/** @brief 按单字符拆分字符串（去首尾空格，跳过空段） */
inline std::vector<std::string> splitBy(const std::string& joined, char delim)
{
    std::vector<std::string> out;
    size_t pos = 0;
    while (pos <= joined.size())
    {
        const auto comma = joined.find(delim, pos);
        auto item = joined.substr(pos, comma == std::string::npos ? std::string::npos : comma - pos);
        const auto first = item.find_first_not_of(" \t");
        if (first != std::string::npos)
        {
            const auto last = item.find_last_not_of(" \t");
            out.push_back(item.substr(first, last - first + 1));
        }
        if (comma == std::string::npos)
            break;
        pos = comma + 1;
    }
    return out;
}

/** @brief 实体串解析结果 */
struct EntityString
{
    uint8_t camp;
    uint8_t squad;
    SigmaModelType type;
    uint32_t model;
};

/** @brief "camp_squad_型号_model" -> 分量（型号名可含 '_'，如 AA_Patriot；非法返回 false） */
inline bool parseEntityString(const std::string& s, EntityString& out)
{
    const auto tokens = splitBy(s, '_');
    if (tokens.size() < 4)
        return false;
    try
    {
        const unsigned long camp  = std::stoul(tokens[0]);
        const unsigned long squad = std::stoul(tokens[1]);
        const unsigned long model = std::stoul(tokens.back());
        if (camp > 0xff || squad > 0xff || model > 0xffff)
            return false;
        std::string name = tokens[2];
        for (size_t i = 3; i + 1 < tokens.size(); ++i)
            name += "_" + tokens[i];
        out.camp  = static_cast<uint8_t>(camp);
        out.squad = static_cast<uint8_t>(squad);
        out.model = static_cast<uint32_t>(model);
        out.type  = tryStringToAssetType(name);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

} // namespace entity_naming

#endif // SIGMA_INTERFACE_ENTITY_NAMING_H
