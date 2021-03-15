/*
Copyright (c) 20121, MACHANI ROBOTICS. All rights reserved.

MACHANI ROBOTICS and its licensors retain all intellectual  perty
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from MACHANI ROBOTICS is strictly prohibited.
*/
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "messages/state.capnp.h"
#include "packages/dynamixel/gems/dynamixel.hpp"
#include "engine/alice/alice_codelet.hpp"
#include "engine/core/optional.hpp"
#include "messages/composite.capnp.h"
#include "packages/composite/gems/parser.hpp"
#include "packages/math/gems/kinematic_tree/kinematic_tree.hpp"

namespace isaac {
namespace dynamixel {

// A motor driver for a Aria's/Sophia's Arms. This codelet receives desired motor
// commands via a message and publishes the current motor state as proto. Multiple
// checks and safe guards are used to protected the driver from misuse. The codelet also currently
// features a debug mode in which individual motors type can be tested with constant speed and position
class DynamixelArmDriver : public alice::Codelet {
 public:
  void start() override;
  void tick() override;
  void stop() override;

  // The desired angular speeds for each motor
  ISAAC_PROTO_RX(CompositeProto, arm_command);
  //ISAAC_PROTO_RX(CompositeProto, finger_command); //TODO
  // The measured angular speeds for each motor
  ISAAC_PROTO_TX(CompositeProto, arm_state);
  //ISAAC_PROTO_TX(CompositeProto, finger_state);   //TODO

  // USB port where Dynamixel controller is located at. usb_port varies depending on the controller
  // device, e.g., "/dev/ttyACM0" or "/dev/ttyUSB0"
  ISAAC_PARAM(std::string, port, "/dev/ttyUSB0");
  // Baud rate of the Dynamixel bus. This is the rate of information transfer.
  ISAAC_PARAM(Baudrate, baudrate, Baudrate::k1M);
  // Model of servo (AX12A, XM430, MX12W, XC430)
  //Abhinav added MX64 and MX106
  ISAAC_PARAM(Model, servo_model, Model::XM430);
  // If set to true dynamixels are controlled in speed mode, otherwise they are controlled in
  // position mode
  ISAAC_PARAM(DynamixelMode, control_mode, DynamixelMode::kPosition);
  // Unique identifier for Dynamixel servos. Each motor needs to be assigned a unique
  // ID using the software provided by Dynamixel. This is a mandatory parameter.
  ISAAC_PARAM(std::vector<int>, servo_ids);
  // Servo maximum torque limit. Caps the amount of torque the servo will apply.
  // 0.0 is no torque, 1.0 is max available torque
  ISAAC_PARAM(double, torque_limit, 1.0);
  // Maximum (absolute) angular speed for wheels
  ISAAC_PARAM(double, max_speed, 4.7);
  // Commands received that are older than command_timeout seconds will be ignored. Kaya will stop
  // if no message is received for command_timeout seconds.
  ISAAC_PARAM(double, command_timeout, 0.3);
  // Enables debug mode in which all motors are driving with constant speed independent from
  // incoming messages. Only in wheel mode, NOT to be used in robotics
  //ISAAC_PARAM(bool, debug_mode, false); 
  // If debug mode is enabled, all motors will rotate with this speed.
  //ISAAC_PARAM(double, debug_speed, 1.0);
  // Enables debug mode in which all motors of one type can be  given a specific motor angle
  // from incoming message. 
  ISAAC_PARAM(bool, debug_position_mode, false);
  // If debug mode is enabled, all motors will reach this position
  ISAAC_PARAM(double, position, 1.0);
  //Speed for servo 
  ISAAC_PARAM(double, speed, 1.0);
  // Name of the node containing the map:KinematicTree component
  ISAAC_PARAM(std::string, kinematic_tree);

 private:
  // Initializes verified servo IDs from configuration
  bool initializeServoIds();
  bool validateKinematicTree(const kinematic_tree::KinematicTree& model);
  // Enable the Dynamixel motors by writing the required values to registers.
  bool enableDynamixels(DynamixelMode mode);
  // Reads commands from message and writes it to motors
  bool receiveCommand(TensorView1d commands);
  // Writes commands to dynamixel motors
  bool writeCommand(TensorConstView1d commands);
  // Reads state from motors and publishes it as message
  void readAndPublishState();
  // Disable the Dynamixel motors in case there is a severe error.
  void disableDynamixels();

  // Failsafe
  alice::Failsafe* failsafe_;
  // Dynamixel
  std::unique_ptr<Dynamixel> dynamixel_;
  // Ids of motors we are working with
  std::vector<uint8_t> servo_ids_;
};

NLOHMANN_JSON_SERIALIZE_ENUM(Model, {
  { Model::AX12A, "AX12A" },
  { Model::XM430, "XM430" },
  { Model::MX12W, "MX12W" },
  { Model::XC430, "XC430" },
  { Model::MX64, "MX64" },
  { Model::MX106, "MX106" },
  { Model::INVALID, nullptr }
});

NLOHMANN_JSON_SERIALIZE_ENUM(Baudrate, {
  {Baudrate::k4_5M, "k4_5M" },
  {Baudrate::k4M, "k4M" },
  {Baudrate::k3M, "k3M" },
  {Baudrate::k2M, "k2M" },
  {Baudrate::k1M, "k1M" },
  {Baudrate::k115200, "k115200" },
  {Baudrate::k57600, "k57600" },
  {Baudrate::k9600, "k9600" },
  {Baudrate::kInvalid, "kInvalid" }
});

NLOHMANN_JSON_SERIALIZE_ENUM(DynamixelMode, {
  {DynamixelMode::kPosition, "position" },
  {DynamixelMode::kVelocity, "velocity" },
  {DynamixelMode::kInvalid, nullptr }
});

}  // namespace dynamixel
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::dynamixel::DynamixelArmDriver);
