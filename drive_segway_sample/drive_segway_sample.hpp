#pragma once

#include "engine/alice/alice.hpp"
#include "messages/state.capnp.h"
#include "messages/basic.capnp.h"
#include "messages/math.capnp.h"
// #include "messages/differential_base.capnp.h"
// #include "messages/imu.capnp.h"
// #include "lib/comm_ctrl_navigation.h"

namespace isaac {

class DriveSegway : public isaac::alice::Codelet {
    public:
    // Has whatever needs to be run in the beginning of the program
    void start() override;
    // Has whatever needs to be run repeatedly
    void tick() override;

    ISAAC_PROTO_RX(BooleanProto, segway_init_success);

    ISAAC_PROTO_TX(StateProto, speed_cmd);
    ISAAC_PROTO_TX(BooleanProto, drive_enable_cmd);

    private:
    char  get_keyboard();
    // char* print_help();
    double set_line_vel;
    double set_angular_vel;
    double line_vel_cmd = 0;
    double angular_vel_cmd = 0;
    uint8_t enable_flag;
    uint8_t pause_flag;
    bool       enable_switch;
    bool    segway_init_ok;
};


} // namespace isaac
ISAAC_ALICE_REGISTER_CODELET(isaac::DriveSegway);