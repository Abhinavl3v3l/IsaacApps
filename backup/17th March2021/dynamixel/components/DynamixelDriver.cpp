/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "DynamixelDriver.hpp"

#include <string>
#include <utility>
#include <vector>

#include "engine/alice/components/Failsafe.hpp"
#include "engine/alice/message.hpp"
#include "engine/core/logger.hpp"
#include "engine/core/tensor/universal_tensor.hpp"
#include "engine/gems/tensor/utils.hpp"
#include "messages/tensor.hpp"
#include "packages/dynamixel/gems/dynamixel.hpp"

namespace isaac
{
  namespace dynamixel
  {

    namespace
    {

      // Max torque limit of dynamixels
      constexpr int kMaxTorqueLimit = 0x3FF;

    } // namespace

    void DynamixelDriver::start()
    {
      // Initialize a failsafe
      failsafe_ = node()->getComponentOrNull<alice::Failsafe>();
      if (failsafe_ == nullptr)
      {
        LOG_WARNING("Running the Dynamixel driver without a failsafe is not recommended");
      }

      // Initialize Dynamixel motors
      dynamixel_.reset(
          new Dynamixel(get_port().c_str(), get_baudrate(), get_servo_model()));

      if (!initializeServoIds())
        return;

      if (!enableDynamixels(get_control_mode()))
        return;
      tickPeriodically();
    }

    void DynamixelDriver::tick()
    {
      Tensor1d command(servo_ids_.size());

      // Special helper debug mode which sends constant speed to motors
      // if (get_debug_mode())
      // {
      //   Fill(command, get_debug_speed());
      // }
      // else
      if (get_debug_position_mode())
      {
        Fill(command, get_position());
      }
      else
      {
        if (!receiveCommand(command.view()))
        {
          Fill(command, 0.0);
        }
      }

      if (!writeCommand(command.const_view()))
      {
        return;
      }

      readAndPublishState();
    }

    void DynamixelDriver::stop()
    {
      disableDynamixels();
    }

    bool DynamixelDriver::initializeServoIds()
    {
      const std::vector<int> config_servo_ids = get_servo_ids();
      Model model = get_servo_model();
      //Debug Purpose
      if (model == Model::XM430)
      {
        LOG_DEBUG("Working on XM430 Actuator");
      }
      else if (model == Model::MX64)
      {
        LOG_DEBUG("Working on MX64 Actuator");
      }
      else if (model == Model::MX106)
      {
        LOG_DEBUG("Working on MX106 Actuator");
      }
      else
      {
        LOG_DEBUG("Working on %i actuator", model);
      }

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
        servo_ids_.push_back(servo_id);
      }

      return true;
    }

    bool DynamixelDriver::enableDynamixels(DynamixelMode mode)
    {
      Model model = get_servo_model();
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
        //Only when using XM430 do we need Control Mode : Position or Velocity
        if (model == Model::XM430) 
        {
          LOG_DEBUG("SETTING CONTROL MODE FOR XM430");
          dynamixel_->setControlMode(servo, mode);
          dynamixel_->writeRegister(servo, RegisterKey::TORQUE_ENABLE, 0);

          
          dynamixel_->writeRegister(servo, RegisterKey::CW_ANGLE_LIMIT, 0);
          dynamixel_->writeRegister(servo, RegisterKey::CCW_ANGLE_LIMIT, 0);

          dynamixel_->writeRegister(servo, RegisterKey::TORQUE_LIMIT,
                                  static_cast<int>(torqueLimit * kMaxTorqueLimit));

          // setting a moving speed can enable torque, so it should be the last step before
          // we manually enable torque
          dynamixel_->writeRegister(servo, RegisterKey::MOVING_SPEED,0);
          dynamixel_->writeRegister(servo, RegisterKey::TORQUE_ENABLE, 1);
        }
        else if(model == Model::MX64){
          LOG_DEBUG("Enabling Dynamixel for MX64");
          dynamixel_->writeRegister(servo, RegisterKey::TORQUE_ENABLE, 0);
          dynamixel_->writeRegister(servo, RegisterKey::CW_ANGLE_LIMIT, 729);
          dynamixel_->writeRegister(servo, RegisterKey::CCW_ANGLE_LIMIT, 3606);

          dynamixel_->writeRegister(servo, RegisterKey::TORQUE_LIMIT,
                                    static_cast<int>(torqueLimit * kMaxTorqueLimit));

          // setting a moving speed can enable torque, so it should be the last step before
          // we manually enable torque
          //Convert this to a function - TOO MUCH CODE
          {
            // Get and clamp speed
            double servo_speed = get_speed();
            double max_speed = get_max_speed();
            if (std::abs(servo_speed) > max_speed)
            {
              servo_speed = std::copysign(max_speed, servo_speed);
            }
            // dynamixel_->writeRegister(servo, RegisterKey::MIN_VOLTAGE_LIMIT, 60);
            // dynamixel_->writeRegister(servo, RegisterKey::MAX_VOLTAGE_LIMIT, 160);

            // Write to motor
            // LOG_DEBUG("Check MOVING_SPEED TESTING 64 %f", dynamixel_->readRegister(servo, RegisterKey::MOVING_SPEED));
            LOG_DEBUG("Updating Speed of Servo MX64  to %f", servo_speed);
            dynamixel_->writeRegister(servo, RegisterKey::MOVING_SPEED,dynamixel_->getAngularSpeedToTicks(servo_speed)); //Return speed of servo at index iToTicks(servo_speed));
          }
          dynamixel_->writeRegister(servo, RegisterKey::TORQUE_ENABLE, 1);
          LOG_DEBUG("Out Of Enabling MX64");
        }
        else if(model == Model::MX106){
          LOG_DEBUG("Enabling Dynamixel for MX106");
          dynamixel_->writeRegister(servo, RegisterKey::TORQUE_ENABLE, 0);
          // dynamixel_->writeRegister(servo, RegisterKey::CW_ANGLE_LIMIT, 471);
          // dynamixel_->writeRegister(servo, RegisterKey::CCW_ANGLE_LIMIT, 2848);

          dynamixel_->writeRegister(servo, RegisterKey::TORQUE_LIMIT,
                                    static_cast<int>(torqueLimit * kMaxTorqueLimit));

          //Adding Speed 
          // setting a moving speed can enable torque, so it should be the last step before
          // we manually enable torque
          {
            // Get and clamp speed
            double servo_speed = get_speed();
            double max_speed = get_max_speed();
            if (std::abs(servo_speed) > max_speed)
            {
              servo_speed = std::copysign(max_speed, servo_speed);
            }

            // Write to motor
            // LOG_DEBUG("Check MOVING_SPEED TESTING 106 %f", dynamixel_->readRegister(servo, RegisterKey::MOVING_SPEED));
            LOG_DEBUG("Updating Speed of Servo MX106 %f", servo_speed);
            dynamixel_->writeRegister(servo, RegisterKey::MOVING_SPEED, dynamixel_->getAngularSpeedToTicks(servo_speed)); //Return speed of servo at index iToTicks(servo_speed));
          }
          
          dynamixel_->writeRegister(servo, RegisterKey::TORQUE_ENABLE, 1);
          LOG_DEBUG("Out Of Enabling");
        }
        /**
     * Abhinav Added 
     **/
        //Debug Purpose
        // dynamixel_->scan();
      }
      return true;
    }

    bool DynamixelDriver::receiveCommand(TensorView1d command)
    {
      if (failsafe_ && !failsafe_->isAlive())
      {
        LOG_DEBUG("failsafe_ : %d and !failsafe_->isAlive : %d", failsafe_, !failsafe_->isAlive());
        LOG_DEBUG("Failsafe is triggered. Stop the robot.");
        // Failsafe is triggered. Stop the robot.
        return false;
      }

      if (!rx_command().available())
      {
        // No command is available. Stop the robot.
        LOG_DEBUG("No command is available. Stop the robot. ");
        return false;
      }

      if (getTickTime() - ToSeconds(rx_command().acqtime()) > get_command_timeout())
      {
        // Command timed out. Stop the robot.
        LOG_DEBUG("Command timed out. Stop the robot. ");
        return false;
      }

      // Parse the state message and verify tensor type
      CpuUniversalTensorConstView pack;
      if (!FromProto(rx_command().getProto().getPack(), rx_command().buffers(), pack))
      {
        reportFailure("Failed to parse command message.");
        return false;
      }
      const auto maybe = pack.tryGet<TensorConstView3d>();
      if (!maybe)
      {
        reportFailure("Received state does not contain a rank 3 tensor of doubles");
        LOG_DEBUG("Received state does not contain a rank 3 tensor of doubles ");
        return false;
      }
      if (maybe->dimensions()[2] != static_cast<int>(servo_ids_.size()))
      {
        reportFailure("Received a command message for %zd motors, but configured %zd motors.",
                      maybe->dimensions()[2], servo_ids_.size());
        LOG_DEBUG("Received a command message for %zd motors, but configured %zd motors.",
                  maybe->dimensions()[2], servo_ids_.size());
        return false;
      }

      // Retreive state
      Copy(maybe->const_slice(0).const_slice(0), command);
      return true;
    }

    bool DynamixelDriver::writeCommand(TensorConstView1d command)
    {
      /**
      * Dynamixel Model- 
      * If dynamixel model is xm430
      *      Check if command is for Velocity mode or Position Mode.
      *      1. Check Operation Mode  using  get_control_mode
      *          1. If Velocity mode run this code 
      *          2. Else if Position mode's code to be added here
      * 
      *      2. Position Mode
      *          1. Handle exception(Angle Value 0 - 4096) - If position value is greater than Max angle.Return
      *          2. Dynamic Speed with Position using Register - VELOCITY_VALUE_OF_PROFILE and GOAL_POSITION
      * 
      * Else if Dynamixel model is MX64 or MX106
      *      1. Call 
      */
      // Send command to motors
      //Get Servo Model
      Model model = get_servo_model();
      switch (model)
      {
      case Model::XM430: //XM430
      {
        DynamixelMode mode = get_control_mode();
        // LOG_DEBUG(" Dynamixel Mode is %d ",mode );
        if (mode == DynamixelMode::kVelocity)
        {
          double max_speed = get_max_speed();
          if (max_speed < 0.0)
          {
            reportFailure("Invalid maximum speed (%f) - sending zero speeds.", max_speed);
            LOG_DEBUG("Invalid maximum speed (%f) - sending zero speeds.", max_speed);
            max_speed = 0.0;
          }

          bool success = true;
          for (size_t i = 0; i < servo_ids_.size(); i++)
          {
            const int servo_id = servo_ids_[i];

            // Get and clamp speed
            double servo_speed = command(i); //Return speed of servo at index i
            // LOG_DEBUG(" Servo Speed : %f and command(%d) is %f",servo_speed,i,command(i));
            if (std::abs(servo_speed) > max_speed)
            {
              servo_speed = std::copysign(max_speed, servo_speed);
            }

            // Write to motor
            LOG_DEBUG("Running in Velocity Mode. %f", servo_speed);
            const bool failed = dynamixel_->writeRegister(servo_id, RegisterKey::MOVING_SPEED,
                                                          dynamixel_->getAngularSpeedToTicks(servo_speed));
            success &= failed;

            // Show with sight
            const std::string key = "motor_" + std::to_string(i + 1);
            show(key + ".command", command(i));
            show(key + ".fail", failed ? 1 : 0);
          }
          // if a write failed, check the error and disable the dynamixels if necessary
          if (!success)
          {
            const byte status = dynamixel_->getStatus();
            const bool serious_error =
                (status & kStatusOverheating) || (status & kStatusOverloaded);

            if (serious_error)
            {
              reportFailure("Dynamixel serious error detected; disabling servos");
              return false;
            }
            // do not log other errors, since the dynamixel API has already logged the error
          }

          return true;
        }
        else if (mode == DynamixelMode::kPosition)
        {
          // LOG_DEBUG(" Dynamixel Mode is %d ",mode );

          bool success = true;
          for (size_t i = 0; i < servo_ids_.size(); i++)
          {
            const int servo_id = servo_ids_[i];

            LOG_DEBUG(" VALUE OF COMMAND FOR SERVO ID %i is %i ",servo_id, command(i));;
            // int current_position  = dynamixel_->readRegister(servo_id, RegisterKey::CURRENT_POSITION);
            // LOG_DEBUG("CURRENT POSITION VALUE %d", current_position);

            unsigned int goal_position = command(i); //Value between 0 - 4095
            LOG_DEBUG(" GOAL POSITION FOR SERVO ID %i is %u ",servo_id, goal_position);
            //Write to motor
            // LOG_DEBUG("Running in Position Mode.");
            const bool failed = dynamixel_->writeRegister(servo_id, RegisterKey::GOAL_POSITION,
                                                          goal_position);
            success &= failed;

            // Show with sight
            const std::string key = "motor_" + std::to_string(i + 1);
            show(key + ".command", command(i));
            show(key + ".fail", failed ? 1 : 0);
          }
          // if a write failed, check the error and disable the dynamixels if necessary
          if (!success)
          {
            const byte status = dynamixel_->getStatus();
            const bool serious_error =
                (status & kStatusOverheating) || (status & kStatusOverloaded);

            if (serious_error)
            {
              reportFailure("Dynamixel serious error detected; disabling servos");
              return false;
            }
            // do not log other errors, since the dynamixel API has already logged the error
          }

          return true;
        }
        else
        {
          LOG_ERROR(" Unknown Operation mode Selected");
          reportFailure(" Dynamixel Unkown Mode Error");
          return false;
        }
      }
      case Model::MX64:
      {
        LOG_DEBUG("Writing Into MX64");
        bool success = true;
        for (size_t i = 0; i < servo_ids_.size(); i++)
        {
          const int servo_id = servo_ids_[i];

          LOG_DEBUG(" VALUE OF COMMAND FOR SERVO ID %i is %i ",servo_id, command(i));
          int current_position  = dynamixel_->readRegister(servo_id, RegisterKey::CURRENT_POSITION);
          LOG_DEBUG("CURRENT POSITION VALUE %d", current_position);

          // unsigned int goal_position = command(i); //Value between 0 - 4095
          unsigned int goal_position = command(i); //Value between 0 - 4095
          //Write to motor
          LOG_DEBUG("Writing Register  for GOAL POSITION ");

          const bool failed = dynamixel_->writeRegister(servo_id, RegisterKey::GOAL_POSITION,
                                                        goal_position);
          success &= failed;

          // Show with sight
          const std::string key = "motor_" + std::to_string(i + 1);
          show(key + ".command", command(i));
          show(key + ".fail", failed ? 1 : 0);
        }
        // if a write failed, check the error and disable the dynamixels if necessary
        if (!success)
        {
          const byte status = dynamixel_->getStatus();
          const bool serious_error =
              (status & kStatusOverheating) || (status & kStatusOverloaded);

          if (serious_error)
          {
            reportFailure("Dynamixel serious error detected; disabling servos");
            return false;
          }
          // do not log other errors, since the dynamixel API has already logged the error
        }

        return true;
      }

      case Model::MX106: //MX106
      {
        bool success = true;
        for (size_t i = 0; i < servo_ids_.size(); i++)
        {
          const int servo_id = servo_ids_[i];

          LOG_DEBUG(" VALUE OF COMMAND FOR SERVO ID %i is %i ",servo_id, command(i));
          // int current_position  = dynamixel_->readRegister(servo_id, RegisterKey::CURRENT_POSITION);
          // LOG_DEBUG("CURRENT POSITION VALUE %d", current_position);

          unsigned int goal_position = command(i); //Value between 0 - 4095
          LOG_DEBUG(" GOAL POSITION IS %u ", goal_position);

          //Write to motor
          // LOG_DEBUG("Running in Position Mode.");
          const bool failed = dynamixel_->writeRegister(servo_id, RegisterKey::GOAL_POSITION,
                                                        goal_position);
          success &= failed;

          // Show with sight
          const std::string key = "motor_" + std::to_string(i + 1);
          show(key + ".command", command(i));
          show(key + ".fail", failed ? 1 : 0);
        }
        // if a write failed, check the error and disable the dynamixels if necessary
        if (!success)
        {
          const byte status = dynamixel_->getStatus();
          const bool serious_error =
              (status & kStatusOverheating) || (status & kStatusOverloaded);

          if (serious_error)
          {
            reportFailure("Dynamixel serious error detected; disabling servos");
            return false;
          }
          // do not log other errors, since the dynamixel API has already logged the error
        }

        return true;
      }
      break;
      default:
      {
        LOG_DEBUG("WRITE TO REGISTER NOT IMPLEMENTED FOR %i MODEL", model);
        return false;
      }
      break;
      }
    }

    void DynamixelDriver::readAndPublishState()
    {
      //Abhinav add
      Model model = get_servo_model();

      switch (model)
      {
      case Model::XM430:
      {
        LOG_DEBUG("XM430 Servo PUBLISH");
        DynamixelMode mode = get_control_mode();
        if (mode == DynamixelMode::kVelocity)
        {
          Tensor3d actual_speed(1, 1, servo_ids_.size());
          for (size_t i = 0; i < servo_ids_.size(); i++)
          {
            const int servo_id = servo_ids_[i];
            const int current_speed_back =
                dynamixel_->readRegister(servo_id, RegisterKey::CURRENT_SPEED);
            actual_speed(0, 0, i) = dynamixel_->getTicksToAngularSpeed(current_speed_back);

            // Visualize with sight
            const std::string key = "motor_" + std::to_string(i + 1);
            show(key + ".state", actual_speed(0, 0, i));
          }
          ToProto(std::move(actual_speed), tx_state().initProto().initPack(), tx_state().buffers());
          tx_state().publish();
        }
        else if (mode == DynamixelMode::kPosition)
        {
          Tensor3d actual_position(1, 1, servo_ids_.size());
          for (size_t i = 0; i < servo_ids_.size(); i++)
          {
            const int servo_id = servo_ids_[i];
            const unsigned int current_position_back =
                dynamixel_->readRegister(servo_id, RegisterKey::CURRENT_POSITION);
            // LOG_DEBUG("Current Position Back %i ", current_position_back);
            actual_position(0, 0, i) = current_position_back;
            // LOG_DEBUG("Actual Position %i ", actual_position(0,0,i));

            // Visualize with sight
            const std::string key = "motor_" + std::to_string(i + 1);
            show(key + ".state", actual_position(0, 0, i));
          }
          ToProto(std::move(actual_position), tx_state().initProto().initPack(), tx_state().buffers());
          tx_state().publish();
        }
        else
        {
          LOG_ERROR(" Unknown Operation mode Selected");
          reportFailure(" Dynamixel Unkown Mode Error");
        }
      }
      break;
      case Model::MX64:
      {
        LOG_DEBUG("MX64 Servo PUBLISH");
        Tensor3d actual_position(1, 1, servo_ids_.size());
        for (size_t i = 0; i < servo_ids_.size(); i++)
        {
          const int servo_id = servo_ids_[i];
          const unsigned int current_position_back =
              dynamixel_->readRegister(servo_id, RegisterKey::CURRENT_POSITION);
          LOG_DEBUG("Current Position Back %i ", current_position_back);
          actual_position(0, 0, i) = current_position_back;
          // LOG_DEBUG("Actual Position %i ", actual_position(0,0,i));

          // Visualize with sight
          const std::string key = "motor_" + std::to_string(i + 1);
          show(key + ".state", actual_position(0, 0, i));
        }
        ToProto(std::move(actual_position), tx_state().initProto().initPack(), tx_state().buffers());
        tx_state().publish();
      }
      break;
      case Model::MX106:
      {
        LOG_DEBUG("MX106 Servo PUBLISH");
        Tensor3d actual_position(1, 1, servo_ids_.size());
        for (size_t i = 0; i < servo_ids_.size(); i++)
        {
          const int servo_id = servo_ids_[i];
          LOG_DEBUG("Reading Register with ID %i", servo_id);
          const unsigned int current_position_back =
              dynamixel_->readRegister(servo_id, RegisterKey::CURRENT_POSITION);
          // LOG_DEBUG("Current Position Back %i ", current_position_back);
          actual_position(0, 0, i) = current_position_back;
          // LOG_DEBUG("Actual Position %i ", actual_position(0,0,i));

          // Visualize with sight
          const std::string key = "motor_" + std::to_string(i + 1);
          show(key + ".state", actual_position(0, 0, i));
        }
        ToProto(std::move(actual_position), tx_state().initProto().initPack(), tx_state().buffers());
        tx_state().publish();
      }
      break;
      default:
        LOG_DEBUG("PUBLISH FOR %i MODEL NOT IMPLEMENTED", model);
        break;
      }
    }

    void DynamixelDriver::disableDynamixels()
    {
      for (const uint8_t servo : servo_ids_)
      {
        // disabling torque needs to be after setting the moving speed, otherwise the servos will
        // automatically enable torque

        dynamixel_->writeRegister(servo, RegisterKey::MOVING_SPEED, 0);
        dynamixel_->writeRegister(servo, RegisterKey::TORQUE_ENABLE, 0);
        
      }
    }

  } // namespace dynamixel
} // namespace isaac
