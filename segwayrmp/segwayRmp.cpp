// #include "engine/core/logger.hpp"
#include <stdio.h>
#include <iostream>
#include "engine/core/math/so2.hpp"
#include "messages/state/differential_base.hpp"
#include "messages/state/common.hpp"
#include "messages/math.hpp"
#include "packages/engine_gems/state/io.hpp"

#include "segwayRmp.hpp"

namespace isaac {
  #define RMPVERSION0_7
  // #define RMPVERSION1_0
  #ifdef  RMPVERSION0_7
  #define  LINESPEEDMPH2MPS     (1/3600.0)
  #define  ANGULARSPEEDMPH2MPS     (1/1000.0)
  #define  IMUACCROBOT2G    (9.8 / 4000.0)
  #define  IMUGYRROBOT2RADPS    (1 / 900.0)
  #endif
  car_speed_data_t SpeedData;
  uint64_t    Speed_TimeStamp;
  uint8_t      Speed_update;

  motor_ticks_t TicksData;
  uint64_t    Ticks_TimeStamp;
  uint8_t     Ticks_update;

  imu_gyr_original_data_t ImuGyrData;
  uint8_t     ImuGyr_update;
  imu_acc_original_data_t ImuAccData;
  uint8_t     ImuAcc_update;
   uint64_t    Imu_TimeStamp;

  odom_pos_xy_t       OdomPoseXy;
  odom_euler_xy_t     OdomEulerXy;
  odom_euler_z_t      OdomEulerZ;
  odom_vel_line_xy_t  OdomVelLineXy;
  uint64_t    Odom_TimeStamp;
  uint8_t     OdomPoseXy_update;
  uint8_t    OdomEulerXy_update;
  uint8_t     OdomEulerZ_update;
  uint8_t     OdomVelLineXy_update;

  set_max_limit_value_fb_t    LimitVelSetFb;
  uint64_t    LimitFb_TimeStamp;
  uint8_t     LimitFb_update;

  uint32_t  chassis_event_id = 0;
  uint8_t    event_update;

  typedef struct{
    uint32_t typeId;
    uint8_t   dataSize;
    uint64_t *segwayTimeStamp;
    uint8_t   * updateFlag;
    void * dataPtr;
  }SegwayData;

  static SegwayData segway_data_tbl[9] = {
    {Chassis_Data_Speed,  sizeof(SpeedData), &Speed_TimeStamp, &Speed_update, &SpeedData},
    {Chassis_Data_Ticks,  sizeof(TicksData), &Ticks_TimeStamp, &Ticks_update, &TicksData},
    {Chassis_Data_Imu_Gyr,  sizeof(ImuGyrData), &Imu_TimeStamp, &ImuGyr_update, &ImuGyrData},
    {Chassis_Data_Imu_Acc,  sizeof(ImuAccData), &Imu_TimeStamp, &ImuAcc_update, &ImuAccData},
    {Chassis_Data_Odom_Pose_xy,  sizeof(OdomPoseXy), &Odom_TimeStamp, &OdomPoseXy_update, &OdomPoseXy},
    {Chassis_Data_Odom_Euler_xy,  sizeof(OdomEulerXy), &Odom_TimeStamp, &OdomEulerXy_update, &OdomEulerXy},
    {Chassis_Data_Odom_Euler_z,  sizeof(OdomEulerZ), &Odom_TimeStamp, &OdomEulerZ_update, &OdomEulerZ},
    {Chassis_Data_Odom_Linevel_xy,  sizeof(OdomVelLineXy), &Odom_TimeStamp, &OdomVelLineXy_update, &OdomVelLineXy},
    {Chassis_Data_Limit_vel_set_fb,  sizeof(LimitVelSetFb), &LimitFb_TimeStamp, &LimitFb_update, &LimitVelSetFb}
  };

 void PubData(StampedBasicFrame_ *frame)
{
  uint8_t i = 0;
  for (; i < sizeof(segway_data_tbl)/sizeof(segway_data_tbl[0]); i++)
  {
    if (frame->type_id == segway_data_tbl[i].typeId) break;
  }
  if (i < sizeof(segway_data_tbl)/sizeof(segway_data_tbl[0])){
    *(segway_data_tbl[i].segwayTimeStamp) = frame->timestamp;
    *(segway_data_tbl[i].updateFlag) =1;
    memcpy( segway_data_tbl[i].dataPtr, frame->data,  segway_data_tbl[i].dataSize);
  }
}

void EvnetPubData(int event_no)
{
  event_update = 1;
  chassis_event_id = event_no;
  LOG_WARNING("Event number from chassis: %d", event_no);
}

void SegwayChassis::start() {
  bool init_ok = true;
  set_comu_interface(comu_serial);// Before calling init_control_ctrl, need to call this function to set whether the communication port is a serial port or a CAN port, "comu)serial":serial; "comu_can":CAN.

  // Necessary to enable GPIO Serial on Jetson AGX Xavier
  set_smart_car_serial("ttyTHS0");

  if (init_control_ctrl() != 0)
  {
      LOG_ERROR("init_control_ctrl() fail!");
      init_ok = false;
      return;
  }
  tickPeriodically();
  timestamp_data.on_new_data = PubData;
  aprctrl_datastamped_jni_register(&timestamp_data);

  event_data.event_callback = EvnetPubData;
  aprctrl_eventcallback_jni_register(&event_data);

  pre_seconds = 0;
  now_seconds = 0;

  auto proto = tx_segway_init_success().initProto();
  proto.setFlag(init_ok);
  tx_segway_init_success().publish();

  // Enable chassis control as part of initialization process
  // carter app does not have logic to enable chassis ctrl
  set_enable_ctrl(1);
}

void SegwayChassis::tick() {
  if (rx_speed_cmd().available()){
    const int64_t time = rx_speed_cmd().acqtime();
    if (!last_speed_cmd_time_ || time > *last_speed_cmd_time_) {
      messages::DifferentialBaseControl speed_command;
      ASSERT(FromProto(rx_speed_cmd().getProto(), rx_speed_cmd().buffers(), speed_command),
            "Failed to parse rx_speed_cmd");
      lineV = speed_command.linear_speed();
      angularV = speed_command.angular_speed();
      last_speed_cmd_time_ = time;
    }
  }

  // Send command to chassis at every tick to maintain command frequency
  // This is to avoid the chassis dropping back into Lock Mode
  // speed_cmd is not guaranteed to have a constant frequency
  set_cmd_vel(lineV, angularV);

  if (rx_load_cmd().available()){
    const int64_t time = rx_load_cmd().acqtime();
    if (!last_load_cmd_time_ || time > *last_load_cmd_time_) {
      auto proto_load_cmd = rx_load_cmd().getProto();
      int16_t load_state = proto_load_cmd.getFlag();
      set_chassis_load_state(load_state);
      last_load_cmd_time_ = time;
    }
  }
  
  if (rx_poweroff_cmd().available()){
    const int64_t time = rx_poweroff_cmd().acqtime();
    if (!last_poweroff_cmd_time_ || time > *last_poweroff_cmd_time_) {
      auto proto_poweroff_cmd = rx_poweroff_cmd().getProto();
      bool  poweroff_command = proto_poweroff_cmd.getFlag();
      if (poweroff_command == true)
      {
        inform_route_poweroff();
      }
      last_poweroff_cmd_time_ = time;
    }
  }

  if (rx_vel_limit_cmd().available()){
    const int64_t time = rx_vel_limit_cmd().acqtime();
    if (!last_vel_limit_cmd_time_ || time > *last_vel_limit_cmd_time_) {
      auto proto_vel_limit_cmd = FromProto(rx_vel_limit_cmd().getProto());
      double linear_forward_max_x  = proto_vel_limit_cmd[0];
      double linear_backward_max_x = proto_vel_limit_cmd[1];
      double angular_max_z = proto_vel_limit_cmd[2];
      set_line_forward_max_vel(linear_forward_max_x);//Set the maximum linear velocity in the direction of advance
      set_line_backward_max_vel(linear_backward_max_x);//Set the maximum linear velocity in the backward direction
      set_angular_max_vel(angular_max_z);//Set the maximum angular velocity
      last_vel_limit_cmd_time_ = time;
    }
  }

  if (rx_remove_push_cmd().available()){
    const int64_t time = rx_remove_push_cmd().acqtime();
    if (!last_remove_push_cmd_time_ || time > *last_remove_push_cmd_time_) {
      auto proto_remove_push_cmd = rx_remove_push_cmd().getProto();
      bool  remove_push_command = proto_remove_push_cmd.getFlag();
      if (remove_push_command == true)
      {
        set_remove_push_cmd();
      }
      last_remove_push_cmd_time_ = time;
    }
  }

  if (rx_hang_cmd().available()){
    const int64_t time = rx_hang_cmd().acqtime();
    if (!last_hang_cmd_time_ || time > *last_hang_cmd_time_) {
      auto proto_hang_cmd = rx_hang_cmd().getProto();
      bool  hang_command = proto_hang_cmd.getFlag();
      if (hang_command == true)
      {
        set_chassis_hang_mode(1);
      }
      else
      {
        set_chassis_hang_mode(0);
      }     
      last_hang_cmd_time_ = time;
    }
  }

  if (Speed_update == 1){
    messages::DifferentialBaseControl speed_fb;
    speed_fb.linear_speed() = (double)SpeedData.car_speed * LINESPEEDMPH2MPS;
    speed_fb.angular_speed() = (double)SpeedData.turn_speed * ANGULARSPEEDMPH2MPS;
    ToProto(speed_fb, tx_speed_fb().initProto(), tx_speed_fb().buffers());
    tx_speed_fb().publish(Speed_TimeStamp);
    Speed_update = 0;
  }

  if (Ticks_update == 1){
    auto proto_ticks_fb = tx_ticks_fb().initProto();
    proto_ticks_fb.setX(TicksData.l_ticks);
    proto_ticks_fb.setY(TicksData.r_ticks);
    tx_ticks_fb().publish(Ticks_TimeStamp);
    Ticks_update = 0;
  }
  if (ImuGyr_update == 1 && ImuAcc_update == 1){
    auto proto_imu_fb = tx_imu_fb().initProto();
    double acc_x = -(double)ImuAccData.acc[1] * IMUACCROBOT2G;
    double acc_y = (double)ImuAccData.acc[0] * IMUACCROBOT2G;
    double acc_z = (double)ImuAccData.acc[2] * IMUACCROBOT2G;
    double gyr_x = -(double)ImuGyrData.gyr[1] * IMUGYRROBOT2RADPS;
    double gyr_y = (double)ImuGyrData.gyr[0] * IMUGYRROBOT2RADPS;
    double gyr_z = (double)ImuGyrData.gyr[2] * IMUGYRROBOT2RADPS;
    proto_imu_fb.setLinearAccelerationX(acc_x);
    proto_imu_fb.setLinearAccelerationY(acc_y);
    proto_imu_fb.setLinearAccelerationZ(acc_z);
    proto_imu_fb.setAngularVelocityX (gyr_x);
    proto_imu_fb.setAngularVelocityY(gyr_y);
    proto_imu_fb.setAngularVelocityZ(gyr_z);
    tx_imu_fb().publish(Imu_TimeStamp);
    ImuGyr_update = 0;
    ImuAcc_update = 0;
  }
  if (OdomPoseXy_update == 1 && OdomEulerXy_update == 1 && OdomEulerZ_update == 1 && OdomVelLineXy_update == 1){
    auto proto_odom_fb = tx_odom_fb().initProto();
    const Pose2d odom_T_robot{SO2d::FromAngle(OdomEulerZ.euler_z),  Vector2d{OdomPoseXy.pos_x, OdomPoseXy.pos_y}};
    ToProto(odom_T_robot, proto_odom_fb.initOdomTRobot());
    const Vector2d speed_data{OdomVelLineXy.vel_line_x, OdomVelLineXy.vel_line_y};
    ToProto(speed_data, proto_odom_fb.initSpeed());
    proto_odom_fb.setAngularSpeed((double)SpeedData.turn_speed * ANGULARSPEEDMPH2MPS);
    proto_odom_fb.setOdometryFrame("segway");
    proto_odom_fb.setRobotFrame("segway");
    tx_odom_fb().publish(Odom_TimeStamp);
    OdomPoseXy_update = 0;
    OdomEulerXy_update = 0;
    OdomEulerZ_update = 0;
    OdomVelLineXy_update = 0;
  }
  if (LimitFb_update == 1){
    auto proto_set_limit_fb = tx_set_limit_fb().initProto();
    proto_set_limit_fb.setX((double)LimitVelSetFb.set_forward_limit_vel_fb * LINESPEEDMPH2MPS);
    proto_set_limit_fb.setY((double)LimitVelSetFb.set_backward_limit_vel_fb * LINESPEEDMPH2MPS);
    proto_set_limit_fb.setZ((double)LimitVelSetFb.set_angular_limit_vel_fb * LINESPEEDMPH2MPS);
    tx_set_limit_fb().publish(LimitFb_TimeStamp);
    LimitFb_update = 0;
  }
  if (event_update == 1){
    auto proto_event_fb = tx_event_fb().initProto();
    proto_event_fb.initCoefficients(1);
    VectorXi coefficients(1);
    coefficients[0] = chassis_event_id;
    ToProto(coefficients, proto_event_fb);
    tx_event_fb().publish();
    event_update = 0;
  }

  now_seconds = ToSeconds(NowCount());
  if (now_seconds > pre_seconds + 1) {
    auto proto_info_fb = tx_info_fb().initProto();
    proto_info_fb.initCoefficients(16);
    VectorXi coefficients(16);
    coefficients[0] = get_version(Host);
    coefficients[1] = get_err_state(Host);
    coefficients[2] = get_version(Motor);
    coefficients[3] = get_err_state(Motor);
    coefficients[4] = get_version(route);
    coefficients[5] = get_err_state(route);
    coefficients[6] = get_bat_soc();
    coefficients[7] = get_bat_charging();
    coefficients[8] = get_bat_mvol();
    coefficients[9] = get_bat_mcurrent();
    coefficients[10] = get_bat_temp();
    coefficients[11] = get_chassis_work_model();
    coefficients[12] = get_vehicle_meter();
    coefficients[13] = get_ctrl_cmd_src();
    coefficients[14] = get_chassis_load_state(); //0: no_load, 1: full_load
    coefficients[15] = get_chassis_mode();//0: lock_mode, 1:ctrl_mode, 2:push_mode, 3:emergency mode, 4:error mode
    ToProto(coefficients, proto_info_fb);
    pre_seconds = now_seconds + 1;
  }
}

void SegwayChassis::stop() {
  set_enable_ctrl(0);
  exit_control_ctrl();
}

}  // namespace isaac
