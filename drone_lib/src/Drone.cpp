#include "drone/Drone.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>

#include "px4_msgs/msg/vehicle_local_position_setpoint.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp" // added
#include "tf2/utils.h"

#include <custom_msgs/msg/position.hpp>

Drone::Drone() : Node("Drone") {

 rclcpp::QoS px4_qos(5);
 px4_qos.best_effort();
 px4_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

 rclcpp::QoS custom_qos(5); // position qos
 custom_qos.best_effort();
 custom_qos.durability(rclcpp::DurabilityPolicy::Volatile);

 rclcpp::QoS telemetry_qos(10); // telemetry logs and status
 telemetry_qos.best_effort();
 telemetry_qos.durability(rclcpp::DurabilityPolicy::Volatile);

 std::string vehicle_id_prefix = "";

 // Essa parte será útil para múltiplos drones
 //if (this->vehicle_id_ != 0) {
 // vehicle_id_prefix = "/px4_" + std::to_string(this->vehicle_id_);
 // this->target_system_ = this->vehicle_id_ + 1;
 //}

 this->vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
 vehicle_id_prefix + "/fmu/out/vehicle_status",
 px4_qos,
 [this](px4_msgs::msg::VehicleStatus::ConstSharedPtr msg) {
 auto set_arm_disarm_reason = [](uint8_t reason)
 {
 DronePX4::ARM_DISARM_REASON value;
 switch (reason) {
 case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_TRANSITION_TO_STANDBY:
 value = DronePX4::ARM_DISARM_REASON::TRANSITION_TO_STANDBY;
 break;
 case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_RC_STICK:
 value = DronePX4::ARM_DISARM_REASON::RC_STICK;
 break;
 case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_RC_SWITCH:
 value = DronePX4::ARM_DISARM_REASON::RC_SWITCH;
 break;
 case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_COMMAND_INTERNAL:
 value = DronePX4::ARM_DISARM_REASON::COMMAND_INTERNAL;
 break;
 case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_COMMAND_EXTERNAL:
 value = DronePX4::ARM_DISARM_REASON::COMMAND_EXTERNAL;
 break;
 case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_MISSION_START:
 value = DronePX4::ARM_DISARM_REASON::MISSION_START;
 break;
 case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_SAFETY_BUTTON:
 value = DronePX4::ARM_DISARM_REASON::SAFETY_BUTTON;
 break;
 case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_AUTO_DISARM_LAND:
 value = DronePX4::ARM_DISARM_REASON::AUTO_DISARM_LAND;
 break;
 case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_AUTO_DISARM_PREFLIGHT:
 value = DronePX4::ARM_DISARM_REASON::AUTO_DISARM_PREFLIGHT;
 break;
 case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_KILL_SWITCH:
 value = DronePX4::ARM_DISARM_REASON::KILL_SWITCH;
 break;
 case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_LOCKDOWN:
 value = DronePX4::ARM_DISARM_REASON::LOCKDOWN;
 break;
 case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_FAILURE_DETECTOR:
 value = DronePX4::ARM_DISARM_REASON::FAILURE_DETECTOR;
 break;
 case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_SHUTDOWN:
 value = DronePX4::ARM_DISARM_REASON::SHUTDOWN;
 break;
 default:
 value = DronePX4::ARM_DISARM_REASON::ARM_DISARM_REASON_NONE;
 }
 return value;
 };

 this->arm_reason_ = set_arm_disarm_reason(msg->latest_arming_reason);
 this->disarm_reason_ = set_arm_disarm_reason(msg->latest_disarming_reason);

 switch (msg->failure_detector_status) {
 case px4_msgs::msg::VehicleStatus::FAILURE_NONE:
 this->failure_ = DronePX4::FAILURE::NONE;
 break;
 case px4_msgs::msg::VehicleStatus::FAILURE_ROLL:
 this->failure_ = DronePX4::FAILURE::ROLL;
 break;
 case px4_msgs::msg::VehicleStatus::FAILURE_PITCH:
 this->failure_ = DronePX4::FAILURE::PITCH;
 break;
 case px4_msgs::msg::VehicleStatus::FAILURE_ALT:
 this->failure_ = DronePX4::FAILURE::ALT;
 break;
 case px4_msgs::msg::VehicleStatus::FAILURE_ARM_ESC:
 this->failure_ = DronePX4::FAILURE::ARM_ESC;
 break;
 case px4_msgs::msg::VehicleStatus::FAILURE_BATTERY:
 this->failure_ = DronePX4::FAILURE::BATTERY;
 break;
 case px4_msgs::msg::VehicleStatus::FAILURE_IMBALANCED_PROP:
 this->failure_ = DronePX4::FAILURE::IMBALANCED_PROP;
 break;
 case px4_msgs::msg::VehicleStatus::FAILURE_MOTOR:
 this->failure_ = DronePX4::FAILURE::MOTOR;
 break;
 case px4_msgs::msg::VehicleStatus::FAILURE_EXT:
 this->failure_ = DronePX4::FAILURE::EXT;
 break;
 default:
 this->failure_ = DronePX4::FAILURE::NONE;
 }

 switch (msg->arming_state) {
 case px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED:
 this->arming_state_ = DronePX4::ARMING_STATE::ARMED;
 break;
 case px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED:
 this->arming_state_ = DronePX4::ARMING_STATE::DISARMED;
 break;
 default:
 this->arming_state_ = DronePX4::ARMING_STATE::DISARMED;
 }

 switch (msg->nav_state) {
 case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MANUAL:
 this->flight_mode_ = DronePX4::FLIGHT_MODE::MANUAL;
 break;
 case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ALTCTL:
 this->flight_mode_ = DronePX4::FLIGHT_MODE::ALTCTL;
 break;
 case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL:
 this->flight_mode_ = DronePX4::FLIGHT_MODE::POSCTL;
 break;
 case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION:
 this->flight_mode_ = DronePX4::FLIGHT_MODE::AUTO_MISSION;
 break;
 case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER:
 this->flight_mode_ = DronePX4::FLIGHT_MODE::AUTO_LOITER;
 break;
 case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_RTL:
 this->flight_mode_ = DronePX4::FLIGHT_MODE::AUTO_RTL;
 break;
 case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ACRO:
 this->flight_mode_ = DronePX4::FLIGHT_MODE::ACRO;
 break;
 case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_TERMINATION:
 this->flight_mode_ = DronePX4::FLIGHT_MODE::TERMINATION;
 break;
 case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD:
 this->flight_mode_ = DronePX4::FLIGHT_MODE::OFFBOARD;
 break;
 case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_STAB:
 this->flight_mode_ = DronePX4::FLIGHT_MODE::STAB;
 break;
 case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF:
 this->flight_mode_ = DronePX4::FLIGHT_MODE::AUTO_TAKEOFF;
 break;
 case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND:
 this->flight_mode_ = DronePX4::FLIGHT_MODE::AUTO_LAND;
 break;
 case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
 this->flight_mode_ = DronePX4::FLIGHT_MODE::AUTO_FOLLOW_TARGET;
 break;
 case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_PRECLAND:
 this->flight_mode_ = DronePX4::FLIGHT_MODE::AUTO_PRECLAND;
 break;
 case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ORBIT:
 this->flight_mode_ = DronePX4::FLIGHT_MODE::ORBIT;
 break;
 case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF:
 this->flight_mode_ = DronePX4::FLIGHT_MODE::AUTO_VTOL_TAKEOFF;
 break;
 case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_DESCEND:
 this->flight_mode_ = DronePX4::FLIGHT_MODE::DESCEND;
 break;
 default:
 this->flight_mode_ = DronePX4::FLIGHT_MODE::UNKNOWN_MODE;
 }
 }
 );

 this->vehicle_timesync_sub_ = this->create_subscription<px4_msgs::msg::TimesyncStatus>(
 vehicle_id_prefix + "/fmu/out/timesync_status",
 px4_qos,
 [this](px4_msgs::msg::TimesyncStatus::ConstSharedPtr msg) {
 this->timestamp_ = std::chrono::time_point<std::chrono::high_resolution_clock>(
 std::chrono::nanoseconds(msg->timestamp));
 }
 );

 this->vehicle_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
 vehicle_id_prefix + "/fmu/out/vehicle_odometry",
 px4_qos,
 [this](px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg) {
 this->odom_timestamp_ = std::chrono::time_point<std::chrono::high_resolution_clock>(
 std::chrono::nanoseconds(msg->timestamp));
 this->ground_speed_ = std::sqrt(
 std::pow(msg->velocity[0], 2) + std::pow(msg->velocity[1], 2));

 this->current_pos_x_ = msg->position[0];
 this->current_pos_y_ = msg->position[1];
 this->current_pos_z_ = msg->position[2];
 this->current_vel_x_ = msg->velocity[0];
 this->current_vel_y_ = msg->velocity[1];
 this->current_vel_z_ = msg->velocity[2];

 // if the quaternion is valid, extract the euler angles for convenience
 if (msg->q[0] != NAN) {
 double y = 0, p = 0, r = 0;
 // the ordering is different: PX4 does wxyz, TF2/Bullet does xyzw
 tf2::getEulerYPR(
 tf2::Quaternion(msg->q[1], msg->q[2], msg->q[3], msg->q[0]),
 y, p, r
 );
 this->yaw_ = static_cast<float>(y);
 this->pitch_ = static_cast<float>(p);
 this->roll_ = static_cast<float>(r);
 }
 }
 );

 // Subscribe to fused global position (lat, lon, alt)
 this->vehicle_global_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
 vehicle_id_prefix + "/fmu/out/vehicle_global_position",
 px4_qos,
 [this](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
 Eigen::Vector3d v;
 v[0] = static_cast<double>(msg->lat); // degrees
 v[1] = static_cast<double>(msg->lon); // degrees
 v[2] = static_cast<double>(msg->alt); // meters AMSL

 std::lock_guard<std::mutex> lk(this->gps_mutex_);
 this->gps_global_position_ = v;
 }
 );

 this->vehicle_airspeed_sub_ = this->create_subscription<px4_msgs::msg::Airspeed>(
 vehicle_id_prefix + "/fmu/out/airspeed",
 px4_qos,
 [this](px4_msgs::msg::Airspeed::ConstSharedPtr msg) {
 this->airspeed_ = msg->true_airspeed_m_s;
 }
 );

 this->vehicle_rates_setpoint_pub_ = this->create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
 vehicle_id_prefix + "/fmu/in/vehicle_rates_setpoint", px4_qos);

 this->vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
 vehicle_id_prefix + "/fmu/in/vehicle_command", px4_qos);

 this->vehicle_trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
 vehicle_id_prefix + "/fmu/in/trajectory_setpoint", px4_qos);

 this->vehicle_offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
 vehicle_id_prefix + "/fmu/in/offboard_control_mode", px4_qos);


 this->position_pub_ = this->create_publisher<custom_msgs::msg::Position>(
 "/telemetry/position", custom_qos);

 // Critical 20Hz position timer for FSM coordination - always active
 this->position_timer_ = this->create_wall_timer(
 std::chrono::milliseconds(50), // 20 Hz - optimized timing
 [this]() {
 custom_msgs::msg::Position msg;

 // NED coordinates
 msg.x_ned = this->current_pos_x_;
 msg.y_ned = this->current_pos_y_;
 msg.z_ned = this->current_pos_z_;
 msg.yaw_ned = this->yaw_;

 msg.vx_ned = this->current_vel_x_;
 msg.vy_ned = this->current_vel_y_;
 msg.vz_ned = this->current_vel_z_;

 // FRD coordinates
 const Eigen::Vector3d positionNED(this->current_pos_x_, this->current_pos_y_, this->current_pos_z_);
 const Eigen::Vector3d positionFRD = this->convertPositionNEDtoFRD(positionNED);

 msg.x_frd = positionFRD.x();
 msg.y_frd = positionFRD.y();
 msg.z_frd = positionFRD.z();
 msg.yaw_frd = this->yaw_ - this->initial_yaw_;

 const Eigen::Vector3d velocityNED(this->current_vel_x_, this->current_vel_y_, this->current_vel_z_);
 const Eigen::Vector3d velocityFRD = this->convertVelocityNEDtoFRD(velocityNED);

 msg.vx_frd = velocityFRD.x();
 msg.vy_frd = velocityFRD.y();
 msg.vz_frd = velocityFRD.z();

 this->position_pub_->publish(msg);
 });

 // Add telemetry publishers
 this->log_pub_ = this->create_publisher<custom_msgs::msg::LogMessage>("/telemetry/logs", telemetry_qos);
 this->drone_status_pub_ = this->create_publisher<custom_msgs::msg::DroneStatus>("/telemetry/drone_status", telemetry_qos);
 
 // Subscribe to battery status
 this->battery_status_sub_ = this->create_subscription<px4_msgs::msg::BatteryStatus>(
 vehicle_id_prefix + "/fmu/out/battery_status", px4_qos,
 [this](const px4_msgs::msg::BatteryStatus::SharedPtr msg) {
 this->battery_voltage_ = msg->voltage_filtered_v;
 });
 
 // Status publisher timer (2Hz)
 this->status_timer_ = this->create_wall_timer(
 std::chrono::milliseconds(500),
 std::bind(&Drone::publishDroneStatus, this));

}

Drone::~Drone(){}

DronePX4::ARMING_STATE Drone::getArmingState() {
 return this->arming_state_;
}

DronePX4::FLIGHT_MODE Drone::getFlightMode() {
 return this->flight_mode_;
}

DronePX4::ARM_DISARM_REASON Drone::getArmReason() {
 return this->arm_reason_;
}

DronePX4::ARM_DISARM_REASON Drone::getDisarmReason() {
 return this->disarm_reason_;
}

DronePX4::FAILURE Drone::getFailure() {
 return this->failure_;
}


Eigen::Vector3d Drone::getLocalPosition() {
 const Eigen::Vector3d posNED(this->current_pos_x_, this->current_pos_y_, this->current_pos_z_);
 const Eigen::Vector3d posFRD = this->convertPositionNEDtoFRD(posNED); 
 return posFRD;
}


float Drone::getAltitude() {
 return this->current_pos_z_;
}


float Drone::getGroundSpeed() {
 return this->ground_speed_;
}

float Drone::getAirSpeed() {
 return this->airspeed_;
}

Eigen::Vector3d Drone::getOrientation() {
 return Eigen::Vector3d({
 this->roll_,
 this->pitch_,
 this->yaw_-initial_yaw_
 });
}

// Implementação do getter thread-safe
Eigen::Vector3d Drone::getGlobalPosition() {
 std::lock_guard<std::mutex> lk(this->gps_mutex_);
 return this->gps_global_position_;
}

void Drone::arm() {
 this->sendCommand(
 px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
 this->target_system_,
 this->target_component_,
 this->source_system_,
 this->source_component_,
 this->confirmation_,
 this->from_external_,
 1.0f
 );
}
void Drone::armSync() {
 while (getArmingState() != DronePX4::ARMING_STATE::ARMED && rclcpp::ok()) {
 this->arm();
 usleep(1e5); // 100 ms
 }
}

void Drone::disarm() {
 this->sendCommand(
 px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
 this->target_system_,
 this->target_component_,
 this->source_system_,
 this->source_component_,
 this->confirmation_,
 this->from_external_,
 0.0f
 );
}

void Drone::disarmSync() {
 while (getArmingState() != DronePX4::ARMING_STATE::DISARMED && rclcpp::ok()) {
 this->disarm();
 usleep(1e5); // 100 ms
 }
}

/*
void Drone::takeoff() {
 this->sendCommand(
 // https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_TAKEOFF
 px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF,
 this->target_system_,
 this->target_component_,
 this->source_system_,
 this->source_component_,
 this->confirmation_,
 this->from_external_,
 0.1f, // Minimum pitch (if airspeed sensor present), desired pitch without sensor (degrees)
 0, // Empty
 0, // Empty
 1.57, // Yaw angle (degrees)
 this->lat_, // Latitude
 this->lon_, // Longitude
 this->alt_ + 5.0f // Altitude (meters)
 ); 
}
*/

void Drone::land()
{
 this->sendCommand(
 px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND,
 this->target_system_,
 this->target_component_,
 this->source_system_,
 this->source_component_,
 this->confirmation_,
 this->from_external_,
 0.1f,
 0,
 0,
 this->yaw_, // orientation
 0.0f,
 0.0f 
 );
}

void Drone::setLocalPosition(float x, float y, float z, float yaw) {

 this->setOffboardControlMode(DronePX4::CONTROLLER_TYPE::POSITION);

 px4_msgs::msg::TrajectorySetpoint msg;

 msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

 const Eigen::Vector3d positionFRD(x, y, z);
 const Eigen::Vector3d positionNED = this->convertPositionFRDtoNED(positionFRD);
 yaw += initial_yaw_;

 msg.position[0] = positionNED.x();
 msg.position[1] = positionNED.y();
 msg.position[2] = positionNED.z();
 msg.yaw = yaw;

 // non-NaN velocity and acceleration fields are used as feedforward terms.
 // We will just set them all to NaN, to keep this API simple.

 msg.velocity[0] = std::numeric_limits<float>::quiet_NaN();
 msg.velocity[1] = std::numeric_limits<float>::quiet_NaN();
 msg.velocity[2] = std::numeric_limits<float>::quiet_NaN();
 msg.yawspeed = std::numeric_limits<float>::quiet_NaN();

 msg.acceleration[0] = std::numeric_limits<float>::quiet_NaN();
 msg.acceleration[1] = std::numeric_limits<float>::quiet_NaN();
 msg.acceleration[2] = std::numeric_limits<float>::quiet_NaN();

 this->vehicle_trajectory_setpoint_pub_->publish(msg);
}

void Drone::setLocalPositionSync(
 double x,
 double y,
 double z,
 double yaw,
 double airspeeed,
 double distance_threshold,
 DronePX4::CONTROLLER_TYPE controller_type)
{
 while (rclcpp::ok()) {
 this->setOffboardControlMode(controller_type);
 this->setLocalPosition(x, y, z, yaw);
 this->setAirSpeed(airspeeed);

 const Eigen::Vector3d currentPosition = getLocalPosition();
 const Eigen::Vector3d goal = Eigen::Vector3d({x,y,z});

 const auto distance = (currentPosition - goal).norm();

 if (distance < distance_threshold) {
 break;
 }

 usleep(1e5); // 100 ms
 }
}

void Drone::setLocalVelocity(float vx, float vy, float vz, float yaw_rate) {
 
 this->setOffboardControlMode(DronePX4::CONTROLLER_TYPE::VELOCITY);
 
 px4_msgs::msg::TrajectorySetpoint msg;

 msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

 msg.position[0] = std::numeric_limits<float>::quiet_NaN();
 msg.position[1] = std::numeric_limits<float>::quiet_NaN();
 msg.position[2] = std::numeric_limits<float>::quiet_NaN();
 msg.yaw = std::numeric_limits<float>::quiet_NaN();

 const Eigen::Vector3d velocityFRD(vx, vy, vz);
 const Eigen::Vector3d velocityNED = this->convertVelocityFRDtoNED(velocityFRD);
 msg.velocity[0] = velocityNED.x();
 msg.velocity[1] = velocityNED.y();
 msg.velocity[2] = velocityNED.z();
 msg.yawspeed = yaw_rate;

 msg.acceleration[0] = std::numeric_limits<float>::quiet_NaN();
 msg.acceleration[1] = std::numeric_limits<float>::quiet_NaN();
 msg.acceleration[2] = std::numeric_limits<float>::quiet_NaN();

 this->vehicle_trajectory_setpoint_pub_->publish(msg);
}

void Drone::setGroundSpeed(float speed) {
 this->setSpeed(speed, true);
}

void Drone::setAirSpeed(float speed) {
 this->setSpeed(speed, false);
}

void Drone::setOffboardControlMode(DronePX4::CONTROLLER_TYPE type) {
 px4_msgs::msg::OffboardControlMode msg;
 msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

 msg.position = false;
 msg.velocity = false;
 msg.acceleration = false;
 msg.attitude = false;
 msg.body_rate = false;
 msg.direct_actuator = false;

 if (type == DronePX4::CONTROLLER_TYPE::POSITION) {
 msg.position = true;
 } else if (type == DronePX4::CONTROLLER_TYPE::VELOCITY) {
 msg.velocity = true;
 } else if (type == DronePX4::CONTROLLER_TYPE::BODY_RATES) {
 msg.body_rate = true;
 } else {
 RCLCPP_WARN(this->get_logger(), "No controller is defined");
 }

 this->vehicle_offboard_control_mode_pub_->publish(msg);
}

void Drone::toOffboardSync() {
 for (int i = 0; i < 20; i++) {
 setLocalPosition(
 current_pos_x_,
 current_pos_y_,
 current_pos_z_,
 std::numeric_limits<float>::quiet_NaN());
 setOffboardControlMode(DronePX4::CONTROLLER_TYPE::POSITION);
 usleep(1e5); // 100 ms
 }
 setOffboardMode();
}

void Drone::setOffboardMode() {
 this->sendCommand(
 px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
 this->target_system_,
 this->target_component_,
 this->source_system_,
 this->source_component_,
 this->confirmation_,
 this->from_external_,
 1.0f,
 6.0f
 );
}

void Drone::toPositionSync() {
 for (int i = 0; i < 10; i++){
 this->sendCommand(
 px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
 this->target_system_,
 this->target_component_,
 this->source_system_,
 this->source_component_,
 this->confirmation_,
 this->from_external_,
 1.0f,
 7.0f
 );
 usleep(1e5);
 }
}

void Drone::setHomePosition(const Eigen::Vector3d& fictual_home) {
 // Variables for Coordinate Systems transformations
 this->frd_home_position_ = fictual_home;
 this->ned_home_position_ = Eigen::Vector3d({current_pos_x_, current_pos_y_, current_pos_z_});
 this->initial_yaw_ = yaw_;
 this->log("INITIAL YAW IS: " + std::to_string(yaw_));
}

void Drone::sendCommand(
 uint32_t command, uint8_t target_system, uint8_t target_component, uint8_t source_system,
 uint8_t source_component, uint8_t confirmation, bool from_external,
 float param1, float param2, float param3,
 float param4, float param5, float param6,
 float param7)
{
 px4_msgs::msg::VehicleCommand msg;
 msg.timestamp = this->get_clock()->now().nanoseconds() / 1000.0;
 msg.command = command;

 msg.param1 = param1;
 msg.param2 = param2;
 msg.param3 = param3;
 msg.param4 = param4;
 msg.param5 = param5;
 msg.param6 = param6;
 msg.param7 = param7;
 msg.confirmation = confirmation;
 msg.source_system = source_system;
 msg.target_system = target_system;
 msg.target_component = target_component;
 msg.from_external = from_external;
 msg.source_component = source_component;

 this->vehicle_command_pub_->publish(msg);
}

void Drone::setSpeed(float speed, bool is_ground_speed)
{
 float speed_type = is_ground_speed ? 1.0f : 0.0f; // true = ground speed, false = air speed
 this->sendCommand(
 // https://mavlink.io/en/messages/common.html#MAV_CMD_DO_CHANGE_SPEED
 px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_CHANGE_SPEED,
 this->target_system_,
 this->target_component_,
 this->source_system_,
 this->source_component_,
 this->confirmation_,
 this->from_external_,
 speed_type, // Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)
 speed, // Speed (-1 indicates no change, -2 indicates return to default vehicle speed)
 -1.0f); // Throttle (-1 indicates no change, -2 indicates return to default throttle value)
}

double Drone::getTime() {
 return this->get_clock()->now().seconds();
}

void Drone::log(const std::string &info) {
 // Existing console logging
 RCLCPP_INFO(this->get_logger(), info.c_str());
 
 // New telemetry logging
 auto log_msg = custom_msgs::msg::LogMessage();
 log_msg.header.stamp = this->get_clock()->now();
 log_msg.node_name = this->get_name();
 log_msg.level = 3; // INFO level
 log_msg.message = info;
 
 log_pub_->publish(log_msg);
}

void Drone::publishDroneStatus() {
 auto status_msg = custom_msgs::msg::DroneStatus();
 status_msg.header.stamp = this->get_clock()->now();
 status_msg.arming_state = static_cast<uint8_t>(this->arming_state_);
 status_msg.flight_mode = static_cast<uint8_t>(this->flight_mode_);
 status_msg.battery_voltage = this->battery_voltage_;
 
 drone_status_pub_->publish(status_msg);
}


//Coordinate System transformations (private functions)

Eigen::Vector3d Drone::convertPositionNEDtoFRD(const Eigen::Vector3d& position_ned) const
{
 // Translate the NED coordinates to be relative to the NED home position
 Eigen::Vector3d translated_position = position_ned - this->ned_home_position_;

 // Apply rotation to account for the initial yaw
 Eigen::Matrix3d rotation;
 rotation << cos(-this->initial_yaw_), -sin(-this->initial_yaw_), 0,
 sin(-this->initial_yaw_), cos(-this->initial_yaw_), 0,
 0, 0, 1;

 // Translate to be relative to the FRD home position
 return rotation * translated_position + this->frd_home_position_;
}

Eigen::Vector3d Drone::convertPositionFRDtoNED(const Eigen::Vector3d& position_frd) const
{
 // Translate the FRD coordinates to be relative to the FRD home position
 Eigen::Vector3d translated_position = position_frd - this->frd_home_position_;

 // Apply inverse rotation to go from FRD to NED coordinates
 Eigen::Matrix3d rotation;
 rotation << cos(this->initial_yaw_), -sin(this->initial_yaw_), 0,
 sin(this->initial_yaw_), cos(this->initial_yaw_), 0,
 0, 0, 1;

 return rotation * translated_position + this->ned_home_position_;
}

Eigen::Vector3d Drone::convertVelocityNEDtoFRD(const Eigen::Vector3d& velocity_ned) const
{
 // Apply rotation to account for the initial yaw
 Eigen::Matrix3d rotation;
 rotation << cos(-this->initial_yaw_), -sin(-this->initial_yaw_), 0,
 sin(-this->initial_yaw_), cos(-this->initial_yaw_), 0,
 0, 0, 1;

 return rotation * velocity_ned;
}

Eigen::Vector3d Drone::convertVelocityFRDtoNED(const Eigen::Vector3d& velocity_frd) const
{
 // Apply inverse rotation to go from FRD to NED coordinates
 Eigen::Matrix3d rotation;
 rotation << cos(this->initial_yaw_), -sin(this->initial_yaw_), 0,
 sin(this->initial_yaw_), cos(this->initial_yaw_), 0,
 0, 0, 1;

 return rotation * velocity_frd;
}