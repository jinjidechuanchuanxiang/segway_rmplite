#pragma once

#include "engine/alice/alice.hpp"
#include "messages/state.capnp.h"
#include "messages/basic.capnp.h"
#include "messages/math.capnp.h"
#include "messages/differential_base.capnp.h"
#include "messages/imu.capnp.h"
#include "lib/comm_ctrl_navigation.h"

namespace isaac {

class SegwayChassis : public isaac::alice::Codelet {
    public:
    // Has whatever needs to be run in the beginning of the program
    void start() override;
    // Has whatever needs to be run repeatedly
    void tick() override;

    ISAAC_PROTO_RX(StateProto, speed_cmd);
    ISAAC_PROTO_RX(BooleanProto, drive_enable_cmd);
    ISAAC_PROTO_RX(BooleanProto, load_cmd);
    ISAAC_PROTO_RX(BooleanProto, poweroff_cmd);
    ISAAC_PROTO_RX(Vector3dProto , vel_limit_cmd);
    ISAAC_PROTO_RX(BooleanProto, remove_push_cmd);
    ISAAC_PROTO_RX(BooleanProto, hang_cmd);

    ISAAC_PROTO_TX(StateProto, speed_fb);
    ISAAC_PROTO_TX(Vector2iProto , ticks_fb);
    ISAAC_PROTO_TX(VectorXiProto, event_fb);
    ISAAC_PROTO_TX(Odometry2Proto, odom_fb);
    ISAAC_PROTO_TX(ImuProto, imu_fb);
    ISAAC_PROTO_TX(Vector3dProto, set_limit_fb);
    ISAAC_PROTO_TX(VectorXiProto, info_fb);
    ISAAC_PROTO_TX(BooleanProto, segway_init_success);

private:
    s_aprctrl_datastamped_t timestamp_data;
    s_aprctrl_event_t                event_data;
    double     pre_seconds;
    double     now_seconds;
    std::optional<int64_t> last_speed_cmd_time_;
    std::optional<int64_t> last_enable_cmd_time_;
    std::optional<int64_t> last_load_cmd_time_;
    std::optional<int64_t> last_poweroff_cmd_time_;
    std::optional<int64_t> last_vel_limit_cmd_time_;
    std::optional<int64_t> last_remove_push_cmd_time_;
    std::optional<int64_t> last_hang_cmd_time_;
};


} // namespace isaac
ISAAC_ALICE_REGISTER_CODELET(isaac::SegwayChassis);