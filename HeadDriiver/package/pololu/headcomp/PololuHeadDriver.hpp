#pragma once

#include <memory>
#include <string>
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "messages/state.capnp.h"
#include "genesis/pololu/gems/maestro.hpp"
#include "packages/composite/gems/measure.hpp"
#include "genesis/dynamixel/gems/dynamixel.hpp"
#include "engine/alice/alice_codelet.hpp"
#include "engine/core/optional.hpp"
#include "messages/composite.capnp.h"
#include "packages/math/gems/kinematic_tree/kinematic_tree.hpp"
#include "packages/composite/gems/parser.hpp"
#include "packages/composite/gems/schema.hpp"
#include "third_party/nlohmann/json.hpp"

namespace isaac
{
  namespace dynamixel
  {
    class Dynamixel;
  }
}
namespace isaac
{
  namespace pololu
  {
    // using namespace dynamixel;
    class PololuHeadDriver : public alice::Codelet
    {
    public:
      void start() override;
      void tick() override;
      void stop() override;

      // The desired angular speeds for each motor
      ISAAC_PROTO_RX(CompositeProto, command);
      // The measured angular speeds for each motor
      ISAAC_PROTO_TX(CompositeProto, state);
      // USB port where pololu controller is located at. usb_port varies depending on the controller
      // device, e.g., "/dev/ttyACM0"
      ISAAC_PARAM(std::string, port, "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT2MZZ48-if00-port0");
      ISAAC_PARAM(std::string, device_12, "/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Mini_Maestro_12-Channel_USB_Servo_Controller_00232818-if00");
      ISAAC_PARAM(std::string, device_24, "/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Mini_Maestro_24-Channel_USB_Servo_Controller_00231982-if00");
      // This is the rate of information transfer.port
      // ISAAC_PARAM(Baudrate, baudrate, Baudrate::k1M);
      // Select the type of pololu in use. Support , MICRO 6, MINI 12,18 and 24
      //More info - https://www.pololu.com/docs/0J40/all#1
      ISAAC_PARAM(Type, pololu_type, Type::MINI_24);
      // Select the type of protocol Compact or  Pololu. Mini SSC(not supported)
      // More info https://www.pololu.com/docs/0J40/all#5.c
      ISAAC_PARAM(Protocol, protocol, Protocol::POLOLU);
      // NOTE : USE THIS PARAMETER FOR TESTING OR DEBUGING, VALUES ARE TO BE TRANSFERE VIA MESSAGE TENSOR.
      // If the channel is configured as a servo, then the target represents
      // the pulse width to transmit in units of quarter-microseconds
      // 0 : Stop sending pulse to Maestro
      // 1500us : 1500 * 4   or 6000
      // Min - Max : Can be set via Wizard,
      // Set the target position of a channel to a given value in 0.25 microsecond units
      // Range :  992 - 2000 in quarter micro-sec or 3968 - 8000 in micro-sec
      // ISAAC_PARAM(double, position, 1500);
      /// Maximum (absolute) angular speed
      // The speed limit is given in units of (0.25 μs)/(10 ms), Speed of 0 is unlimited speed.
      // If we set speed to 140 or 3.5us/ms, Moving target from 1000us to 1350us will take 100ms.
      ISAAC_PARAM(double, pspeed, 0);
      ISAAC_PARAM(double, pmax_speed, 0);
      // The acceleration limit is a value from 0 to 255 in units of (0.25 μs)/(10 ms)/(80 ms)
      // Value of 0 is no acceleration limit, Range - {0-255} units
      ISAAC_PARAM(double, pacceleration, 0);
      ISAAC_PARAM(double, pmax_acceleration, 0);
      // Each pololu device controller cab be assigned a ID or Device number
      ISAAC_PARAM(double, device_number, 12.0); //12 default device number
      // Moving state of servos
      // ISAAC_PARAM(bool, is_moving, false);
      // Channels of Maestro for servos.
      // ISAAC_PARAM(std::vector<(unsigned char)>, channels);
      // Error - https://www.pololu.com/docs/0J40/all#4.e
      // ISAAC_PARAM(Error, error, Error::OK);

      //Dynamixel Parameters
      // ISAAC_PARAM(dynamixel::Baudrate, baudrate, dynamixel::Baudrate::k1M);
      // ISAAC_PARAM(dynamixel::DynamixelMode, control_mode, dynamixel::DynamixelMode::kPosition);
      ISAAC_PARAM(std::vector<int>, servo_ids);
      ISAAC_PARAM(double, torque_limit, 1.0);
      ISAAC_PARAM(double, max_speed, 4.7);
      ISAAC_PARAM(double, command_timeout, 0.3);
      ISAAC_PARAM(double, position, 1.0);
      ISAAC_PARAM(double, speed, 1.0);
      // ISAAC_PARAM(std::string, kinematic_tree);

    private:
      //State Schema is composite proto schema for arm of
      // Generates the composite schema for publishing state
      void initStateSchema();
      // Generates a schema for parsing command
      void initCommandParser();
      // Parses command from Composite message
      void parseCommand();
      // Queries current arm state
      void publishState();

      //Dynamixel
      // Initializes verified servo IDs from configuration
      bool initializeServoIds();
      bool enableDynamixels(dynamixel::DynamixelMode mode);
      // Reads commands from message and writes it to motors
      bool receiveCommand(TensorView1d commands);
      // Writes commands to dynamixel moleators
      bool writeCommand(TensorConstView1d commands);
      // Disable the Dynamixel motors in case there is a servo error.
      void disableDynamixels();

      // Initialize home position for arm
      void goHome();

      //Initialize the type of Pololu
      // void init_Type(Type);
      // std::vector<std::string> channel_names_24 = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11",
      //                                              "12", "13", "14", "15", "16", "17", "18", "19", "20", "21", "22", "23"};
      // std::vector<std::string> channel_names_12 = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11"};
      //Channel or ID's of servos in series Dynamixel, Pololu24 and Pololu12
      std::vector<std::string> channel_names_ = {"d3", "d4", "d5", "d6", "d7",
                                                 "0_1", "1_1", "2_1", "3_1", "4_1", "5_1", "6_1", "7_1", "8_1", "9_1", "10_1", "11_1",
                                                 "12_1", "13_1", "14_1", "15_1", "16_1", "17_1", "18_1", "19_1", "20_1", "21_1", "22_1", "23_1",
                                                 "0_2", "1_2", "2_2", "3_2", "4_2", "5_2", "6_2", "7_2", "8_2", "9_2", "10_2", "11_2"};
      Maestro device12;
      Maestro device24;
      Maestro device;
      // Acquire time for most recently received position command
      std::optional<int64_t> last_command_time_;
      // Parser for maestro command
      composite::Parser command_parser_12;
      composite::Parser command_parser_24;
      composite::Parser command_parser_;
      // Cached schema for state messageMaestro device;
      composite::Schema state_schema_;
      std::vector<u_short> commands_;
      MaestroConfig config12;
      MaestroConfig config24;
      

      //Dynamixel
      std::unique_ptr<dynamixel::Dynamixel> dynamixel_430;
      // Ids of motors we are working with
      std::vector<uint8_t> servo_ids_ = {3,4,5,6,7};
      // Failsafe
      alice::Failsafe *failsafe_;
    };
    NLOHMANN_JSON_SERIALIZE_ENUM(Type, {{Type::MICRO_6, "MICRO_6"},
                                        {Type::MINI_12, "MINI_12"},
                                        {Type::MINI_18, "MINI_18"},
                                        {Type::MINI_24, "MINI_24"}});

    NLOHMANN_JSON_SERIALIZE_ENUM(Protocol, {{Protocol::COMPACT, "COMPACT"},
                                            {Protocol::MINI_SSC, "MINI_SSC"},
                                            {Protocol::POLOLU, "POLOLU"}});
    NLOHMANN_JSON_SERIALIZE_ENUM(dynamixel::Model, {{dynamixel::Model::AX12A, "AX12A"},
                                         {dynamixel::Model::XM430, "XM430"},
                                         {dynamixel::Model::MX12W, "MX12W"},
                                         {dynamixel::Model::XC430, "XC430"},
                                         {dynamixel::Model::MX64, "MX64"},
                                         {dynamixel::Model::MX106, "MX106"},
                                         {dynamixel::Model::INVALID, nullptr}});

    // NLOHMANN_JSON_SERIALIZE_ENUM(dynamixel::Baudrate, {{dynamixel::Baudrate::k4_5M, "k4_5M"},
    //                                         {dynamixel::Baudrate::k4M, "k4M"},
    //                                         {dynamixel::Baudrate::k3M, "k3M"},
    //                                         {dynamixel::Baudrate::k2M, "k2M"},
    //                                         {dynamixel::Baudrate::k1M, "k1M"},
    //                                         {dynamixel::Baudrate::k115200, "k115200"},
    //                                         {dynamixel::Baudrate::k57600, "k57600"},
    //                                         {dynamixel::Baudrate::k9600, "k9600"},
    //                                         {dynamixel::Baudrate::kInvalid, "kInvalid"}});

    NLOHMANN_JSON_SERIALIZE_ENUM(dynamixel::DynamixelMode, {{dynamixel::DynamixelMode::kPosition, "position"},
                                                 {dynamixel::DynamixelMode::kVelocity, "velocity"},
                                                 {dynamixel::DynamixelMode::kInvalid, nullptr}});
  } //namespace pololu
} //namespace isaac
ISAAC_ALICE_REGISTER_CODELET(isaac::pololu::PololuHeadDriver);