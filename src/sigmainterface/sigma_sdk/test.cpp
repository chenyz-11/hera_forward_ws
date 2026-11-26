#include "dxfd_protocol/dxfd_msg_id.h"
#include "sigmaSDK.h"
#include "sigmaAssist.h"
#include <bitset>

#define REMOTE_IP      "192.168.31.243"
#define REMOTE_PORT    9990
#define LOCAL_PORT     26530

using namespace std;

// 声明绑定的回调函数
void onRecvModelInfo(const dxfd_message_t& msg);
void onRecvTargetFusionData(const dxfd_message_t& msg);

// 主函数入口
int main(int argc, char* argv[])
{
    int sendMsgCount = 0;                // 发送消息计数（未使用）
    SigmaSDKDemo sigmaSDK(REMOTE_IP, REMOTE_PORT, LOCAL_PORT);  // 初始化SDK，绑定IP和端口

    // 按需绑定所需的回调函数，用于处理接收到的仿真系统数据
    sigmaSDK.registerCallback(DXFD_MSG_ID_MODEL_INFO, [](const dxfd_message_t& msg) { onRecvModelInfo(msg); });
    sigmaSDK.registerCallback(DXFD_MSG_ID_TARGET_FUSION_DATA, [](const dxfd_message_t& msg) { onRecvTargetFusionData(msg); });
    sigmaSDK.registerCallback(DXFD_MSG_ID_ORIGIN_GLOBAL_POSITION, [](const dxfd_message_t& msg) { printf("origin global position is known\n"); });
    sigmaSDK.registerCallback(DXFD_MSG_ID_ALG_HEARTBEAT, [](const dxfd_message_t& msg) { printf("algorithm heartbeat is received\n"); });
    sigmaSDK.registerCallback(DXFD_MSG_ID_WEAPON_SPEC, [](const dxfd_message_t& msg) {
        dxfd_weapon_spec_t spec;
        dxfd_msg_weapon_spec_decode(&msg, &spec);
        printf("recv weapon spec, entityID:%ld ammoID:%u totalCount:%u range:%f damage:%f name:%s\n",
               spec.entity_id, spec.ammoID, spec.totalCount, spec.range, spec.damage, spec.name);
    });
    sigmaSDK.registerCallback(DXFD_MSG_ID_WEAPON_FIRE_ACK, [](const dxfd_message_t& msg) {
        dxfd_weapon_fire_ack_t ack;
        dxfd_msg_weapon_fire_ack_decode(&msg, &ack);
        printf("recv weapon fire ack, seq:%u resultCode:%u projectileID:%ld ammoID:%u ammoRemain:%u\n",
               ack.seq, ack.resultCode, ack.projectileID, ack.ammoID, ack.ammoRemain);
    });
    sigmaSDK.registerCallback(DXFD_MSG_ID_STRIKE_RESULT, [](const dxfd_message_t& msg) {
        dxfd_strike_result_t result;
        dxfd_msg_strike_result_decode(&msg, &result);
        printf("recv strike result, shooterID:%ld targetID:%ld resultType:%u hitCount:%u damage:%f\n",
               result.shooterID, result.targetID, result.resultType, result.hitCount, result.damageValue);
    });

    int64_t id = getEntityID(SigmaModelType::BTR82, 0, 0, 1);

    // 循环监听键盘输入，发送对应指令
    while(true)
    {
        char c = getchar();              // 获取键盘输入字符
        switch(c)                       // 根据字符执行不同发送逻辑
        {
            case '1':
                printf("send request sim data msg\n");        // 打印提示：请求仿真数据
                //按模型名称查询  查询内容：<阵营_小队_类型_ID>   阵营:0为红方，1为蓝方 | 小队：默认填0 | 类型_ID:仿真软件界面中显示的名称，不区分大小写
                sigmaSDK.SendRequestSimData(MODEL_NAME, "0_0_BTR82_33");  // 发送仿真数据请求

                //按模型类型查询，并返回场景中所有该类型的模型信息 类型包括：UAV/WheelVehicle/TrackVehicle/Ship/People/StaticMesh/Poi  不区分大小写
                //sigmaSDK.SendRequestSimData(MODEL_TYPE, "uav");
                break;

            case '2':             
            {
                printf("send motion command msg\n");          // 打印提示：发送运动控制指令
                sigmaSDK.SendMotionCommand(id, 20, 402340082, 1162331826, 0);  // 发送模型运动控制指令
                printf("发送成功, entityID: %s\n", bitset<64>(id).to_string().c_str());
                break;
            }

            case '3':
            {
                printf("send set model tags msg\n");          // 打印提示：设置模型标签
                sigmaSDK.SendSetModelTags(id, "test");  // 给模型设置标签（test）
                break;
            }

            case '4':
                printf("send spawn model msg\n");             // 打印提示：生成/刷新模型
                sigmaSDK.SendSpawnModel("BTR82", 0, 0, 33, 1, 402335600, 1162331790, 0, 0, 0, 0, 1, 1, 1);  // 生成指定模型并设置位置姿态
                printf("发送成功 entityID: %s\n", bitset<64>(id).to_string().c_str());
                break;

            case '5':
                printf("send request sim data msg by tag\n");        // 打印提示：请求仿真数据
                //按模型名称查询  查询内容：<阵营_小队_类型_ID>   阵营:0为红方，1为蓝方 | 小队：默认填0 | 类型_ID:仿真软件界面中显示的名称，不区分大小写
                sigmaSDK.SendRequestSimData(TAG, "test");  // 发送仿真数据请求
                break;

            case '6':
                printf("send weapon fire cmd\n");        // 打印提示：发送开火命令
                // 开火命令：shooterID 开火平台, targetID 目标(=0 按坐标), ammoID 弹药, fireMode 单发, salvoCount 数量, seq 序号, fireDelayMs 延时(ms), 目标坐标, 提前量
                sigmaSDK.SendWeaponFireCmd(id, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0);
                break;

            case '7':
                printf("send loadout query\n");        // 打印提示：发送载荷查询
                // 载荷查询：entityID 目标实体（应答为多条 weapon_spec）
                sigmaSDK.SendLoadoutQuery(id);
                break;

            case '8':
                printf("send velocity command\n");        // 打印提示：发送速度控制指令
                // 速度控制：entityID, flags(DXFD_VEL_FLAG), seq, vx, vy, vz, yawDeg, yawRateDegS, ax, ay, az
                sigmaSDK.SendVelocityCommand(id, DXFD_VEL_FLAG_VELOCITY, 0, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                break;
        }
    }
    return 0;
}

// 回调函数实现逻辑
void onRecvModelInfo(const dxfd_message_t& msg)
{
    dxfd_model_info_t modelInfo;
    dxfd_msg_model_info_decode(&msg, &modelInfo);
    printf("recv model info, entityID:%ld, location:%d,%d alt:%f size:%f,%f,%f health:%f\n",
                    modelInfo.entityID, modelInfo.locationLat, modelInfo.locationLon, modelInfo.locationAlt,
                    modelInfo.sizeX, modelInfo.sizeY, modelInfo.sizeZ, modelInfo.healthValue);
}

void onRecvTargetFusionData(const dxfd_message_t& msg)
{
    dxfd_target_fusion_data_t targetFusionData;
    dxfd_msg_target_fusion_data_decode(&msg, &targetFusionData);
    printf("recv target fusion data, entityID:%ld, location:%d,%d alt:%f size:%f,%f,%f\n",
                    targetFusionData.targetEntityID, targetFusionData.locationLat, targetFusionData.locationLon, targetFusionData.locationAlt,
                    targetFusionData.sizeX, targetFusionData.sizeY, targetFusionData.sizeZ);
}
