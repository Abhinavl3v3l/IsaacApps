
#include <string>
#include <utility>
#include <vector>

#include "PololuDriver.hpp"

#include "engine/core/assert.hpp"
#include "engine/alice/message.hpp"
#include "engine/core/logger.hpp"
#include "engine/core/tensor/universal_tensor.hpp"
#include "engine/gems/tensor/utils.hpp"
#include "messages/tensor.hpp"
#include "genesis/pololu/gems/maestro.hpp"
#include "engine/core/logger.hpp"
#include "packages/map/KinematicTree.hpp"

namespace isaac
{
    namespace pololu
    {
        namespace
        {
            constexpr int kNumChannels = 12;
            
        }
        void PololuDriver::start()
        {
            LOG_DEBUG("POLOLU DRIVER START");
            //TODO
            // LOG_DEBUG("POLOLU TYPE %i", get_pololu_type());
            // LOG_DEBUG("POLOLU Protocol %i",get_protocol());
            // LOG_DEBUG("POLOLU Device File %s", get_device().c_str());
            // LOG_DEBUG("POLOLU Devce Number %f", int(get_device_number()));
            // config = MaestroConfig();
            

            config = MaestroConfig(get_pololu_type(), get_protocol(), get_device().c_str(), static_cast<unsigned char>(get_device_number()));
            LOG_DEBUG("Configs  Type:  %i, Protocol %i, Device File %s, Device Number %i",
                      config.getType(), config.getProtocol(), config.getDeviceFile().c_str(), config.getDeviceNumber());
            //Error Checks to be handled here
            // init_Type(get_pololu_type());
            device = Maestro(config);
            device.connect();
            initStateSchema();
            tickPeriodically();
        }
        void PololuDriver::tick()
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
    // void init_Type(Type type)
    // {
    //     if (type == Type::MICRO_6)
    //     {
    //         kNumChannels = 6;
    //         channel_names_ = {"0", "1", "2", "3", "4", "5"};
    //     }
    //     else if (type == Type::MINI_12)
    //     {
    //         kNumChannels = 12;
    //         cannel_names_ = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11"};
    //     }
    //     else if (type == Type::MINI_18)
    //     {
    //         kNumChannels = 18;
    //         channel_names_ = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16", "17"};
    //     }
    //     else if (type == Type::MINI_24)
    //     {
    //         kNumChannels = 24;
    //         channel_names_ = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16", "17", "18", "19", "20", "21", "22", "23"};
    //     }
    //     else
    //     {
    //         LOG_DEBUG("INVALID POLOLU TYPE SELECTED");
    //     }
    // }
    void PololuDriver::initStateSchema()
    {
        LOG_DEBUG("POLOLU DRIVER - INIT STATE SCHEMA");
        std::vector<composite::Quantity> quantities;
        for (int i = 0; i < kNumChannels; i++)
        {
            quantities.push_back(
                composite::Quantity::Scalar(channel_names_[i], composite::Measure::kPosition)); //Need 12 Channel names
            quantities.push_back(composite::Quantity::Scalar(channel_names_[i], composite::Measure::kSpeed));
        }
        state_schema_ = composite::Schema(std::move(quantities));
        LOG_DEBUG("STATE SCHEMA INITIATED");
    }

    void PololuDriver::initCommandParser()
    {
        LOG_DEBUG("POLOLU DRIVER - INIT STATE SCHEMA");
        command_parser_.requestSchema(composite::Schema(channel_names_, composite::Measure::kPosition));
        LOG_DEBUG("POLOLU DRIVER - STATE SCHEMA INITIATED");
        return;
    }

    // Parses command from Composite message or Read command and write to pololu
    void PololuDriver::parseCommand()
    {
        LOG_DEBUG("parseCommand START");
        initCommandParser();

        VectorXd command(kNumChannels);
        if (!command_parser_.parse(rx_command().getProto(), rx_command().buffers(), command))
        {
            reportFailure("Fails to parse joint command");
            return;
        }
        commands_.clear();
        commands_.reserve(channel_names_.size());

        //This was needed as we had to convert our commands first and then send, use commands_ vector after conversion
        /*u_short goal_position = command(0);
            commands_.push_back(goal_position);*/
        for (size_t i = 0; i < channel_names_.size(); i++)
        {
            commands_.push_back((u_short)(command(i)));
        }
        /* Send command to Driver*/
        // bool success = true;
        for (size_t i = 0; i < channel_names_.size(); i++)
        {
            // LOG_DEBUG("Maestro Channel id %s, Command given %i ", channel_names_[i].c_str(), commands_[i]);
            // LOG_DEBUG("Setting target with channel values(of i) = %i and command give = %i ", i, commands_[i]);
            device.setTargetOnChannel(i, commands_[i]); //driver call
        }
        //Call to Error Function
        //Error Handling

        for (int i = 0; i < 7; i++)
        {
            show("Maestro_servo" + std::to_string(i + 1) + ".command", command(i));
        }
    }

    void PololuDriver::publishState()
    {
        LOG_DEBUG("INSIDE PUBLISH STATE ");
        const int64_t acqtime = getTickTimestamp();
        int offset = 0;
        u_short state;
        Tensor1d state_data(kNumChannels);
        for (int i = 0; i < kNumChannels; i++)
        {
            device.getPositionOnChannel(i, state);
            // LOG_DEBUG("STATE FROM POLOLU : %u", state);
            state_data(offset++) = state;

            const std::string state = "state_motor_" + std::to_string(i + 1);
            show(state + ".state", state);
        }
        auto arm_proto_builder = tx_state().initProto();
        composite::WriteSchema(state_schema_, arm_proto_builder);
        ToProto(std::move(state_data), arm_proto_builder.initValues(), tx_state().buffers());
        tx_state().publish(acqtime);
    }

    void PololuDriver::stop()
    {
        device.disconnect();
    }
}
}