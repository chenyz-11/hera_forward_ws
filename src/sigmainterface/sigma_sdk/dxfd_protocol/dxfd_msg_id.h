#ifndef DXFD_MSG_ID_H
#define DXFD_MSG_ID_H

#define SINGLE_LOWER_PC_MAX_NODE_COUNT 200
#define QUERY_CONTENT_LEN 128

enum DXFD_MSG_ID{
    DXFD_MSG_ID_HEARTBEAT = 0, 
    DXFD_MSG_ID_VEHICLE_INITIAL_CONDITION, 
    DXFD_MSG_ID_ENVIRONMENT_PARAMS,
    DXFD_MSG_ID_RENDER_DATA,
    DXFD_MSG_ID_SIMULATION_CONTROL,
    DXFD_MSG_ID_FILE_OPERATOR,
    DXFD_MSG_ID_SENSOR_NOISE,
    DXFD_MSG_ID_TRIGGER_INFO,
    DXFD_MSG_ID_MULTI_RENDER_DATA,
    DXFD_MSG_ID_MODEL_INFO,                    
    DXFD_MSG_ID_REQUEST_SIM_DATA,       
    DXFD_MSG_ID_MOTION_COMMAND,     
    DXFD_MSG_ID_SET_MODEL_TAG,             
    DXFD_MSG_ID_SPAWN_MODEL,
    DXFD_MSG_ID_FAULT_CMD,
    DXFD_MSG_ID_TARGET_FUSION_DATA,
    DXFD_MSG_ID_ORIGIN_GLOBAL_POSITION,
    DXFD_MSG_ID_FORMATION_TYPE,
    DXFD_MSG_ID_ALG_MODE,
    DXFD_MSG_ID_TARGET_INFO,
    DXFD_MSG_ID_ALG_PARAM,
    DXFD_MSG_ID_UAV_ROLE,
    DXFD_MSG_ID_ALG_WAYPOINTS,
    DXFD_MSG_ID_ASSIGNMENT_RESULT,
    DXFD_MSG_ID_FAIL_MASK,
    DXFD_MSG_ID_ALGORITHM_CONTROL_CMD,
    DXFD_MSG_ID_SYSTEM_CONTROL_CMD,
    DXFD_MSG_ID_SYSTEM_CONTROL_CMD_ACK,
    DXFD_MSG_ID_UAV_MISSION_STATE,
    DXFD_MSG_ID_ALG_HEARTBEAT,
    DXFD_MSG_ID_CAMERA_CONTROL,
    DXFD_MSG_ID_REFERENCE_TRAJECTORY,
    DXFD_MSG_ID_WEAPON_SPEC,
    DXFD_MSG_ID_LOADOUT_QUERY,
    DXFD_MSG_ID_WEAPON_FIRE_CMD,
    DXFD_MSG_ID_WEAPON_FIRE_ACK,
    DXFD_MSG_ID_STRIKE_RESULT,
    // Current SigmaPI5 Windows receiver dispatches the 43-byte velocity packet at 38.
    // Keep this explicit: inserting/removing earlier enum entries must not change the wire ID.
    DXFD_MSG_ID_VELOCITY_COMMAND = 38,
};

enum SIM_COMPONENT_ID{
    COMPONENT_ALL = 0,
    COMPONENT_PX4,
    COMPONENT_JSB,
    COMPONENT_RC,
};

enum SENSOR_TYPE{
    AIRSPEED = 0,
    GPS,
    IMU,
    MAG,
    BARO,
};

enum PARAM_TYPE {
    PARAM_ALL,
    PARAM_POSITION,
    PARAM_ATTITUDE,
    PARAM_VELOCITY,
    PARAM_TERRAIN,
    PARAM_WINDSPEED,
    PARAM_OTHERS,
};

enum DXFD_SIM_MODE{
    SITL_SENSOR = 0,
    SITL_STATE,
    HITL_STATE,
};

enum DXFD_SIM_CONTROL_CMD_TYPE{
    IDLE = 0,
    RUN_COMPONENT_SOFTWARE,
    STOP_COMPONENT_SOFTWARE,
    SET_PARAMS,
    REBOOT_ALL_SOFTWARE,
    STOP_ALL_SOFTWARE,
};

enum DXFD_QUERY_METHOD {
    MODEL_TYPE = 0,
    MODEL_NAME,
    TAG,
};

enum DXFD_MODEL_TYPE {
    UAV = 0,
    WHEEL_VEHICLE,
    TRACK_VEHICLE,
    SHIP,
    PEOPLE,
    STATIC_MESH,
    POI,
};

enum WEAPON_TYPE {
    WEAPON_CANNON = 0,   // 航炮
    WEAPON_MISSILE,      // 导弹
    WEAPON_ROCKET,       // 火箭弹
    WEAPON_BOMB,         // 炸弹
    WEAPON_TORPEDO,      // 鱼雷
    WEAPON_LASER,        // 定向能/激光
};

enum GUIDANCE_TYPE {
    GUIDANCE_NONE = 0,      // 无制导（自由弹道）
    GUIDANCE_ACTIVE,        // 主动寻的
    GUIDANCE_SEMI_ACTIVE,   // 半主动
    GUIDANCE_COMMAND,       // 指令制导
};

enum WEAPON_SPEC_KIND {
    WEAPON_KIND_LAUNCHER = 0,   // 武器（发射装置）
    WEAPON_KIND_AMMO,           // 弹药
};

enum FIRE_MODE {
    FIRE_SINGLE = 0,   // 单发
    FIRE_BURST,        // 连发
    FIRE_SALVO,        // 齐射
};

enum FIRE_RESULT_CODE {
    FIRE_RESULT_OK = 0,        // 成功
    FIRE_RESULT_NO_TARGET,     // 无目标
    FIRE_RESULT_NO_AMMO,       // 无弹药
    FIRE_RESULT_OUT_OF_RANGE,  // 射程外
    FIRE_RESULT_OUT_OF_FOV,    // 射界外
    FIRE_RESULT_NOT_READY,     // 武器未就绪
};

enum STRIKE_RESULT_TYPE {
    STRIKE_HIT = 0,        // 命中
    STRIKE_MISS,           // 脱靶
    STRIKE_INTERCEPTED,    // 被拦截
    STRIKE_DUD,            // 未命中已落地/失效
};

#endif //DXFD_MSG_ID_H
