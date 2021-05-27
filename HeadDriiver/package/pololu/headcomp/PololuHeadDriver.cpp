
#include <string>
#include <utility>
#include <vector>
#include <fstream>

#include "PololuHeadDriver.hpp"

#include "engine/core/assert.hpp"
#include "engine/alice/message.hpp"
#include "engine/core/logger.hpp"
#include "engine/core/tensor/universal_tensor.hpp"
#include "engine/gems/tensor/utils.hpp"
#include "messages/tensor.hpp"
#include "genesis/pololu/gems/maestro.hpp"
#include "engine/core/logger.hpp"


namespace isaac
{
    namespace pololu
    {
        namespace
        {
            constexpr int kNumChannels_12 = 12;
            constexpr int kNumChannels_24 = 24;
            constexpr int kMaxTorqueLimit = 0x3FF;
            constexpr int kNumJoints = 7;
            constexpr int kNumChannels_ = 41; //Dyanmixel(ID : 3,4,5,6,7) + Maestro24  + Meastro12  = 41
        }

        /**
         * @brief 
         * Start of head drivers
         */
        void PololuHeadDriver::start()
        {
            // Connect to Device with device configuration
            config12 = MaestroConfig(get_pololu_type(), get_protocol(), get_device_12().c_str(), static_cast<unsigned char>(get_device_number()));
            config24 = MaestroConfig(get_pololu_type(), get_protocol(), get_device_24().c_str(), static_cast<unsigned char>(get_device_number()));
            // LOG_DEBUG("Configs  Type:  %i, Protocol %i, Device File %s, Device Number %i",
            //           config.getType(), config.getProtocol(), config.getDeviceFile().c_str(), config.getDeviceNumber());//check

            /**
             * @brief 
             * TODO - Error Checks
             */
            dynamixel_430.reset(
                new dynamixel::Dynamixel(get_port().c_str(), dynamixel::Baudrate::k1M, dynamixel::Model::XM430));

            if (!initializeServoIds())
                return;

            if (!enableDynamixels(dynamixel::DynamixelMode::kPosition))
                return;

            //Initialize and connect to device
            device24 = Maestro(config24);
            device12 = Maestro(config12);
            device24.connect();
            device12.connect();
            goHome();
            initStateSchema(); //This handles both pololu and dynamixel servos's schema
            tickPeriodically();
        }

        /**
         * @brief 
         *  Per tick
         */
        void PololuHeadDriver::tick()
        {
            LOG_DEBUG("INSIDE TICK");
            if (rx_command().available())
            {
                LOG_DEBUG("RECEIVED A COMMAND");
                const int64_t time = rx_command().acqtime();
                if (!last_command_time_ || time > *last_command_time_)
                {
                    // If message is the first received message (last_command_time_ not initialized) or a more
                    // recent command, parse and send to arm.
                    last_command_time_ = time;
                    parseCommand();
                }
                else
                {
                    LOG_DEBUG("COMMAND TIME INCONSISTENCY");
                }
            }
            else
            {
                LOG_DEBUG("NO COMMAND RECEIVED");
            }
            publishState();
        }

        void PololuHeadDriver::goHome()
        {
            Json j;
            std::cout<< "Testing Go Home"  << std::endl;

            std::ifstream i("/workspaces/genesis/genesis/pololu/headcomp/test.json");
            
            i >> j;
            LOG_DEBUG("Json Object");
            std::cout<< "Home Object : "   << j << std::endl;

            std::vector<int> vec = j;

            // for (u_int i = 0; i < test_vec.size(); i++)
            // {
            //     LOG_DEBUG("Value  of  %u: %i ",i, test_vec[i]);
            // }



            bool success = true;
            //manually enter the values for each of the 5 servos
            bool failed = dynamixel_430->writeRegister(3, dynamixel::RegisterKey::GOAL_POSITION, vec[0]);
            success &= failed;
            failed = dynamixel_430->writeRegister(4, dynamixel::RegisterKey::GOAL_POSITION, vec[1]);
            success &= failed;
            failed = dynamixel_430->writeRegister(5, dynamixel::RegisterKey::GOAL_POSITION, vec[2]);
            success &= failed;
            failed = dynamixel_430->writeRegister(6, dynamixel::RegisterKey::GOAL_POSITION, vec[3]);
            success &= failed;
            failed = dynamixel_430->writeRegister(7, dynamixel::RegisterKey::GOAL_POSITION, vec[4]);
            success &= failed;
            if (!success)
            {
                const byte status_430 = dynamixel_430->getStatus();
                const bool serious_error_430 =
                    (status_430 & dynamixel::kStatusOverheating) || (status_430 & dynamixel::kStatusOverloaded);
                if (serious_error_430)
                {
                    reportFailure("Dynamixel_430 serious error detected; disabling servos");
                    return;
                }
            }

            for (size_t i = 5; i < channel_names_.size() - 13; i++)
            {
                LOG_DEBUG("Maestro Channel id %u, Command given %i ", i - 5, vec[i]);
                device24.setTargetOnChannel(i - 5, vec[i]); //driver call
            }
            for (size_t i = 29; i < channel_names_.size(); i++)
            {
                LOG_DEBUG("Maestro Channel id %u, Command given %i ", i - 29, vec[i]);
                device12.setTargetOnChannel(i - 29, vec[i]); //driver call
            }

            //Call to Error Function

        }

        bool PololuHeadDriver::initializeServoIds()
        {
            //get servo ids list from json
            const std::vector<int> config_servo_ids = get_servo_ids();
            servo_ids_.clear();
            servo_ids_.reserve(config_servo_ids.size());
            for (size_t i = 0; i < config_servo_ids.size(); i++)
            {
                const int servo_id = config_servo_ids[i];
                // Check that IDs are in the range [0, 255]. 254 is broadcast to all
                if (servo_id < 0 || 255 < servo_id)
                {
                    reportFailure("Invalid Dynamixel servo ID: %d", servo_id);
                    return false;
                }
                // Check that IDs are unique
                for (size_t j = i + 1; j < config_servo_ids.size(); j++)
                {
                    if (servo_id == config_servo_ids[j])
                    {
                        reportFailure("Invalid duplicated Dynamixel servo ID: %d", servo_id);
                        return false;
                    }
                }
                LOG_DEBUG("SERVO %d added", servo_id);
                servo_ids_.push_back(servo_id);
            }

            return true;
        }
        bool PololuHeadDriver::enableDynamixels(dynamixel::DynamixelMode mode)
        {
            double torqueLimit = get_torque_limit();
            if (torqueLimit < 0.0 || 1.0 < torqueLimit)
            {
                reportFailure("Torque limit of %f is out of range. Must be in [0,1]");
                return false;
            }
            else
            {
                LOG_DEBUG("Torqe Value:  %f ", torqueLimit);
            }

            for (const uint8_t servo : servo_ids_)
            {

                LOG_DEBUG("SETTING CONTROL MODE FOR XM430's for servo %u", servo);
                dynamixel_430->setControlMode(servo, mode);
                dynamixel_430->writeRegister(servo, dynamixel::RegisterKey::TORQUE_ENABLE, 0);
                dynamixel_430->writeRegister(servo, dynamixel::RegisterKey::CW_ANGLE_LIMIT, 0);
                dynamixel_430->writeRegister(servo, dynamixel::RegisterKey::CCW_ANGLE_LIMIT, 0);

                dynamixel_430->writeRegister(servo, dynamixel::RegisterKey::TORQUE_LIMIT,
                                             static_cast<int>(torqueLimit * kMaxTorqueLimit));

                // setting a moving speed can enable torque, so it should be the last step before
                // we manually enable torque
                dynamixel_430->writeRegister(servo, dynamixel::RegisterKey::MOVING_SPEED, 0);
                dynamixel_430->writeRegister(servo, dynamixel::RegisterKey::TORQUE_ENABLE, 1);
            }
            return true;
        }

        void PololuHeadDriver::initStateSchema()
        {
            LOG_DEBUG("POLOLU HEAD DRIVER - INIT STATE SCHEMA");
            std::vector<composite::Quantity> quantities;
            //instead of this add all three type of motors state

            for (int i = 0; i < kNumChannels_; i++)
            {
                LOG_DEBUG("Adding to channel %i value %s", i, channel_names_[i].c_str());
                quantities.push_back(
                    composite::Quantity::Scalar(channel_names_[i], composite::Measure::kPosition));
                quantities.push_back(composite::Quantity::Scalar(channel_names_[i], composite::Measure::kSpeed));
            }
            LOG_DEBUG(" BEFORE setting state schema ");
            state_schema_ = composite::Schema(std::move(quantities));
            LOG_DEBUG("STATE SCHEMA INITIATED");
        }

        void PololuHeadDriver::initCommandParser()
        {
            LOG_DEBUG("POLOLU HEAD DRIVER - INIT STATE SCHEMA");
            // Type type = get_pololu_type();
            command_parser_.requestSchema(composite::Schema(channel_names_, composite::Measure::kPosition));
            LOG_DEBUG("POLOLU HEAD DRIVER - STATE SCHEMA INITIATED");
            return;
        }

        // Parses command from Composite message or Read command and write to pololu
        void PololuHeadDriver::parseCommand()
        {
            LOG_DEBUG("parseCommand START");
            // Type type = get_pololu_type();
            // float conv = 11.375;
            initCommandParser();
            VectorXd command(kNumChannels_);

            if (!command_parser_.parse(rx_command().getProto(), rx_command().buffers(), command))
            {
                reportFailure("Fails to parse joint command");
                return;
            }
            commands_.clear();
            commands_.reserve(channel_names_.size());

            // commands_ will have 41 values
            //Dynamixel - First 5 commands are for dynamixel

            // unsigned int goal_position = static_cast<float>(RadToDeg(command(0))) * conv;
            unsigned int goal_position = command(0);
            commands_.push_back(goal_position);

            // goal_position = static_cast<float>(RadToDeg(command(1))) * conv;
            goal_position = command(1);
            commands_.push_back(goal_position);

            // goal_position = static_cast<float>(RadToDeg(command(2))) * conv;
            goal_position = command(2);
            commands_.push_back(goal_position);

            // goal_position = static_cast<float>(RadToDeg(command(3))) * conv;
            goal_position = command(3);
            commands_.push_back(goal_position);

            // goal_position = static_cast<float>(RadToDeg(command(4))) * conv;
            goal_position = command(4);
            commands_.push_back(goal_position);

            for (size_t i = 5; i < channel_names_.size() - 12; i++)
            {
                commands_.push_back((u_short)(command(i)));
            }

            for (size_t i = 29; i < channel_names_.size(); i++)
            {
                commands_.push_back((u_short)(command(i)));
            }

            for (size_t i = 0; i < channel_names_.size(); i++)
            {
                LOG_DEBUG("Command value of  %u is %i", i, commands_[i]);
            }
            //Set Target to Servos
            //Dynamixel
            bool success = true;
            //manually enter the values for each of the 5 servos
            bool failed = dynamixel_430->writeRegister(3, dynamixel::RegisterKey::GOAL_POSITION, commands_[0]);
            success &= failed;
            failed = dynamixel_430->writeRegister(4, dynamixel::RegisterKey::GOAL_POSITION, commands_[1]);
            success &= failed;
            failed = dynamixel_430->writeRegister(5, dynamixel::RegisterKey::GOAL_POSITION, commands_[2]);
            success &= failed;
            failed = dynamixel_430->writeRegister(6, dynamixel::RegisterKey::GOAL_POSITION, commands_[3]);
            success &= failed;
            failed = dynamixel_430->writeRegister(7, dynamixel::RegisterKey::GOAL_POSITION, commands_[4]);
            success &= failed;
            if (!success)
            {
                const byte status_430 = dynamixel_430->getStatus();
                const bool serious_error_430 =
                    (status_430 & dynamixel::kStatusOverheating) || (status_430 & dynamixel::kStatusOverloaded);
                if (serious_error_430)
                {
                    reportFailure("Dynamixel_430 serious error detected; disabling servos");
                    return;
                }
            }

            for (size_t i = 5; i < channel_names_.size() - 13; i++)
            {
                LOG_DEBUG("Maestro Channel id %u, Command given %i ", i - 5, commands_[i]);
                device24.setTargetOnChannel(i - 5, commands_[i]); //driver call
            }
            for (size_t i = 29; i < channel_names_.size(); i++)
            {
                LOG_DEBUG("Maestro Channel id %u, Command given %i ", i - 29, commands_[i]);
                device12.setTargetOnChannel(i - 29, commands_[i]); //driver call
            }

            //Call to Error Function
            //Error Handling

            for (unsigned int i = 0; i < channel_names_.size(); i++)
            {
                if (i < 5)
                {
                    show("Dynamixel Servo" + std::to_string(i + 1) + ".command", command(i));
                }
                else
                {
                    show("Pololu Servo" + std::to_string(i + 1) + ".command", command(i));
                }
            }
        }

        void PololuHeadDriver::publishState()
        {
            LOG_DEBUG("INSIDE PUBLISH STATE ");
            // Type type = get_pololu_type();
            const int64_t acqtime = getTickTimestamp();
            int offset = 0;
            u_short state;
            Tensor1d state_data(2 * kNumChannels_);

            //Dynamixel States
            for (size_t i = 0; i < servo_ids_.size(); i++)
            {
                const int servo_id = servo_ids_[i];
                unsigned int current_position_back =
                    dynamixel_430->readRegister(servo_id, dynamixel::RegisterKey::CURRENT_POSITION);
                //Value received should be in radians (between -2pi and 2pi ?)
                // current_position_back = static_cast<float>(DegToRad((current_position_back * 0.0878906)));
                const int current_speed_back =
                    dynamixel_430->readRegister(servo_id, dynamixel::RegisterKey::CURRENT_SPEED);

                LOG_DEBUG("Current Position Back %i of servo id %i ", current_position_back, servo_id);
                LOG_DEBUG("Current Speed Back %i ", current_speed_back);

                state_data(offset++) = current_position_back;
                state_data(offset++) = current_speed_back;
                // Visualize with sight
                const std::string key_pos = "pos_motor_" + std::to_string(i + 1);
                show(key_pos + ".state", current_position_back);
                const std::string key_speed = "speed_motor_" + std::to_string(i + 1);
                show(key_speed + ".state", current_speed_back);
            }

            for (int i = 0; i < kNumChannels_24; i++)
            {
                device24.getPositionOnChannel(i, state);
                LOG_DEBUG("STATE FROM POLOLU 1 : %u", state);
                state_data(offset++) = state;

                const int current_speed_back = get_pspeed();
                state_data(offset++) = current_speed_back;

                const std::string state = "state_motor_" + std::to_string(i + 1);
                show(state + ".state", state);
                const std::string key_speed = "speed_motor_" + std::to_string(i + 1);
                show(key_speed + ".state", current_speed_back);
            }
            for (int i = 0; i < kNumChannels_12; i++)
            {
                device12.getPositionOnChannel(i, state);
                LOG_DEBUG("STATE FROM POLOLU 2 : %u", state);
                state_data(offset++) = state;
                const int current_speed_back = get_pspeed();
                state_data(offset++) = current_speed_back;

                const std::string state = "state_motor_" + std::to_string(i + 1);
                show(state + ".state", state);
                const std::string key_speed = "speed_motor_" + std::to_string(i + 1);
                show(key_speed + ".state", current_speed_back);
            }
            auto head_proto_builder = tx_state().initProto();
            composite::WriteSchema(state_schema_, head_proto_builder);
            ToProto(std::move(state_data), head_proto_builder.initValues(), tx_state().buffers());
            tx_state().publish(acqtime);
        }

        void PololuHeadDriver::stop()
        {
            disableDynamixels();
            device24.connect();
            device12.connect();
        }

        void PololuHeadDriver::disableDynamixels()
        {
            for (const uint8_t servo : servo_ids_)
            {
                // disabling torque needs to be after setting the moving speed, otherwise the servos will
                // automatically enable torque(
                LOG_DEBUG("Disabling Servo ID %i", servo);

                dynamixel_430->writeRegister(servo, dynamixel::RegisterKey::MOVING_SPEED, 0);
                if (dynamixel_430->readRegister(servo, dynamixel::RegisterKey::TORQUE_ENABLE) == 1)
                {
                    LOG_DEBUG("TORQUE IS ENABLED");
                }
                dynamixel_430->writeRegister(servo, dynamixel::RegisterKey::TORQUE_ENABLE, 0);
            }
        }
    }
}