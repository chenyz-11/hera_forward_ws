#ifndef SIGMA_ASSIST_H
#define SIGMA_ASSIST_H

#include <iostream>
#include <string>
#include <stdexcept>
#include <cstdint>


enum class SigmaModelType : uint16_t
{
    BigUAV = 0,
    MiddleUAV = 1,
    SmallUAV = 2,
    BigUGV = 3,
    MiddleUGV = 4,
    SmallUGV = 5,
    Challenger2 = 6,
    ArmySoldier = 7,
    M1A2 = 8,
    RPGSoldier = 9,
    M5 = 10,
    M2A3 = 11,
    BTR82 = 12,
};

inline std::string AssetTypeToString(SigmaModelType type)
{
    switch (type)
    {
        case SigmaModelType::BigUAV: return "BigUAV";
        case SigmaModelType::MiddleUAV: return "MiddleUAV";
        case SigmaModelType::SmallUAV: return "SmallUAV";
        case SigmaModelType::BigUGV: return "BigUGV";
        case SigmaModelType::MiddleUGV: return "MiddleUGV";
        case SigmaModelType::SmallUGV: return "SmallUGV";
        case SigmaModelType::Challenger2: return "Challenger2";
        case SigmaModelType::ArmySoldier: return "ArmySoldier";
        case SigmaModelType::M1A2: return "M1A2";
        case SigmaModelType::RPGSoldier: return "RPGSoldier";
        case SigmaModelType::M5: return "M5";
        case SigmaModelType::M2A3: return "M2A3";
        case SigmaModelType::BTR82: return "BTR82";
        default: throw std::invalid_argument("Unknown SigmaModelType");
    }
}

inline SigmaModelType StringToAssetType(const std::string& str)
{
    if (str == "BigUAV") return SigmaModelType::BigUAV;
    if (str == "MiddleUAV") return SigmaModelType::MiddleUAV;
    if (str == "SmallUAV") return SigmaModelType::SmallUAV;
    if (str == "BigUGV") return SigmaModelType::BigUGV;
    if (str == "MiddleUGV") return SigmaModelType::MiddleUGV;
    if (str == "SmallUGV") return SigmaModelType::SmallUGV;
    if (str == "Challenger2") return SigmaModelType::Challenger2;
    if (str == "ArmySoldier") return SigmaModelType::ArmySoldier;
    if (str == "M1A2") return SigmaModelType::M1A2;
    if (str == "RPGSoldier") return SigmaModelType::RPGSoldier;
    if (str == "M5") return SigmaModelType::M5;
    if (str == "M2A3") return SigmaModelType::M2A3;
    if (str == "BTR82") return SigmaModelType::BTR82;
    throw std::invalid_argument("Unknown asset name string: " + str);
}

inline int64_t getEntityID(SigmaModelType type, uint8_t campID, uint8_t squadID, uint32_t modelID)
{
    int64_t Res = 0;
    Res += (int64_t)type << 48;
	Res += (int64_t)campID << 40;
	Res += (int64_t)squadID << 32;
	Res += modelID;
    return Res;
}

inline void parseEntityID(const int64_t entityID, SigmaModelType& type, uint8_t& campID, uint8_t& squadID, uint32_t& modelID)
{
    type = static_cast<SigmaModelType>(entityID >> 48);
	campID = entityID >> 40 & 0xff;
	squadID = entityID >> 32 & 0xff;
	modelID = entityID & 0xffff;
}

inline uint8_t setTrajFlags(const bool yaw_deg, const bool speed_ms, const bool time_s, const bool curvature)
{
    uint8_t flags = 0;
    if (yaw_deg) flags |= 0x01;
    if (speed_ms) flags |= 0x02;
    if (time_s) flags |= 0x04;
    if (curvature) flags |= 0x08;   
    return flags;
}

struct trajPoint{
    float x_cm, y_cm, z_cm;
    float yaw_deg, speed_ms, time_s, curvature;
};

#endif //SIGMA_ASSIST_H