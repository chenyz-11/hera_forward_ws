#include <cstddef>
#include "dxfd_protocol/dxfd_msg_velocity_command.h"

static_assert(DXFD_MSG_ID_VELOCITY_COMMAND == 38, "SigmaPI5 Windows velocity opcode");
static_assert(sizeof(dxfd_velocity_command_t) == 43, "packed velocity wire length");
static_assert(offsetof(dxfd_velocity_command_t, vx_m_s) == 11, "packed wire layout");

int main()
{
    dxfd_message_t message{};
    auto length = dxfd_msg_velocity_command_encode(&message, 0, 0, 1, 3, 17,
        .6f, 0.f, .2f, 0.f, 0.f, 0.f, 0.f, 0.f);
    dxfd_velocity_command_t decoded{};
    dxfd_msg_velocity_command_decode(&message, &decoded);
    return length == 51 && message.msgID == 38 && message.length == 43 &&
           message.checksum == calculateChecksum(&message) && decoded.entity_id == 1 &&
           decoded.seq == 17 && decoded.flags == 3 && decoded.vx_m_s == .6f &&
           decoded.vz_m_s == .2f ? 0 : 1;
}
