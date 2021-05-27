#pragma once

#include <memory>
#include <string>
#include <vector>


#include "engine/alice/alice_codelet.hpp"
#include "messages/state.capnp.h"
#include "genesis/pololu/gems/maestro.hpp"
#include "engine/alice/alice_codelet.hpp"
#include "engine/core/optional.hpp"
#include "messages/composite.capnp.h"
#include "packages/math/gems/kinematic_tree/kinematic_tree.hpp"
#include "packages/composite/gems/parser.hpp"
#include "packages/composite/gems/schema.hpp"
#include "packages/composite/gems/measure.hpp"
namespace isaac{
    namespace pololu{

        class PololuDriver : public alice::Codelet
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
            ISAAC_PARAM(std::string, device, "/dev/ttyACM0");
            // This is the rate of information transfer.
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
            // ISAAC_PARAM(double, speed, 0); 
            // ISAAC_PARAM(double, max_speed,0);
            // The acceleration limit is a value from 0 to 255 in units of (0.25 μs)/(10 ms)/(80 ms)
            // Value of 0 is no acceleration limit, Range - {0-255} units
            // ISAAC_PARAM(double, acceleration, 0);
            // ISAAC_PARAM(double, max_acceleration, 0);
            // Each pololu device controller cab be assigned a ID or Device number
            ISAAC_PARAM(double, device_number, 12.0); //12 default device number
            // Moving state of servos
            // ISAAC_PARAM(bool, is_moving, false);
            // Channels of Maestro for servos.
            // ISAAC_PARAM(std::vector<(unsigned char)>, channels);
            // Error - https://www.pololu.com/docs/0J40/all#4.e
            // ISAAC_PARAM(Error, error, Error::OK);   
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
            //Initialize the type of Pololu
            // void init_Type(Type);
            std::vector<std::string> channel_names_ = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11"};

            Maestro device;
            // Acquire time for most recently received position command
             std::optional<int64_t> last_command_time_;
            // Parser for maestro command
            composite::Parser command_parser_;
             // Cached schema for state message
            composite::Schema state_schema_;
            std::vector<u_short> commands_;
             MaestroConfig config;

             

        };
        NLOHMANN_JSON_SERIALIZE_ENUM(Type, {{Type::MICRO_6, "MICRO_6"},
                                            {Type::MINI_12, "MINI_12"},
                                            {Type::MINI_18, "MINI_18"},
                                            {Type::MINI_24, "MINI_24"}});

        NLOHMANN_JSON_SERIALIZE_ENUM(Protocol, {{Protocol::COMPACT, "COMPACT"},
                                                {Protocol::MINI_SSC, "MINI_SSC"},
                                                {Protocol::POLOLU, "POLOLU"}});
    } //namespace pololu_maestro
} //namespace isaac
ISAAC_ALICE_REGISTER_CODELET(isaac::pololu::PololuDriver);