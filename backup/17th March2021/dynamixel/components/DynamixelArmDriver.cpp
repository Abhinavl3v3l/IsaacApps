#include "DynamixelArmDriver.hpp"

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
#include "packages/composite/gems/measure.hpp"
#include "packages/composite/gems/parser.hpp"
#include "packages/map/KinematicTree.hpp"

// Will receive a composite message with either of the following
// Joint Angles
// Joint Speed
// 3D Pose

//Will publish  a state schema with joint velocity and speed.

namespace isaac
{
  namespace dynamixel
  {
    namespace
    {
      // Max torque limit of dynamixels
      constexpr int kMaxTorqueLimit = 0x3FF;
      constexpr int kNumJoints = 7;
      // [106, 106, 430, 64, 430 ,430, 430]
      const int servos = {9, 11, 15, 13, 17, 19, 21};
    }

    /**
     *  1. Load Kinematic File, check kinematic file
     *  2. Initialize State Schema for outgoing message/publish state
     *  3. Set to initial state/home state.
     * 
     **/
    void DynamixelArmDriver::start()
    {
      // Initialize a failsafe
      failsafe_ = node()->getComponentOrNull<alice::Failsafe>();
      if (failsafe_ == nullptr)
      {
        LOG_WARNING("Running the Dynamixel driver without a failsafe is not recommended");
      }
      // Try to get the map::KinematicTree component
      const auto maybe_kinematic_tree_node = try_get_kinematic_tree();
      if (!maybe_kinematic_tree_node)
      {
        reportFailure("KinematicTree node is not specified.");
        return;
      }
      const auto maybe_kinematic_tree_component =
          node()->app()->getNodeComponentOrNull<map::KinematicTree>(*maybe_kinematic_tree_node);
      if (!maybe_kinematic_tree_component)
      {
        reportFailure("Node %s does not contain KinematicTree component.",
                      maybe_kinematic_tree_node->c_str());
        return;
      }

      // Get the kinematic_tree object and validate against hardware
      const kinematic_tree::KinematicTree &model = maybe_kinematic_tree_component->model();
      if (!validateKinematicTree(model))
      {
        reportFailure("Kinematic tree model does not match hardware.");
        return;
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

    /**
     *    
     * 1. Receive command as Composite Message
     *    1.Check Command available
     *    2. Parse Command 
     *      1. Initialize command parser
     *      2. Create a schema with respect to operation mode 
     *      3. parseCommand() 
     *        1. Set control mode
     *        2. Based on mode Initialize command parser for that command mode.
     *            1. Request a schema with joint names and 
     * 2. Publish Command  as Composite Message 
     * 
     **/
    void DynamixelDriver::tick()
    {

      /*
      Tensor1d command(servo_ids_.size());

      // Special helper debug mode which sends constant speed to motors
      if (get_debug_mode())
      {
        Fill(command, get_debug_speed());
      }
      else if (get_debug_position_mode())
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

      readAndPublishState();*/

      if (rx_arm_command().available())
      {
        const int64_t time = rx_arm_command().acqtime();
        if (!last_command_time_ || time > *last_command_time_)
        {
          // If message is the first received message (last_command_time_ not initialized) or a more
          // recent command, parse and send to arm.
          last_command_time_ = time;
          parseCommand();
        }
      }
    }

    void DynamixelArmDriver::stop()
    {
      disableDynamixels();
    }

    bool DynamixelArmDriver::initializeServoIds()
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
        LOG_DEBUG("SERVO %d added", servo_id)
        servo_ids_.push_back(servo_id);
      }

      return true;
    }
    //Currently Set for only Left Arm
    bool DynamixelArmDriver::goHome()
    {
      bool success = true;
      for (const uint8_t servo : servo_ids_)
      {
        if (servo == 13 || servo == 17 || servo == 19 || servo == 21)
        {
          if (servo == 13)
          {
            const bool failed = dynamixel_->writeRegister(servo, RegisterKey::GOAL_POSITION, 2115.0);
            success &= failed;
          }
          if (servo == 17)
          {
            const bool failed = dynamixel_->writeRegister(servo, RegisterKey::GOAL_POSITION, 2028);
            success &= failed;
          }
          if (servo == 19)
          {
            const bool failed = dynamixel_->writeRegister(servo, RegisterKey::GOAL_POSITION, 1970.0);
            success &= failed;
          }
          if (servo == 21)
          {
            const bool failed = dynamixel_->writeRegister(servo, RegisterKey::GOAL_POSITION, 2083);
            success &= failed;
          }
        }
        if (servo == 9 || servo == 11)
        {
          if (servo == 9)
          {
            const bool failed = dynamixel_->writeRegister(servo, RegisterKey::GOAL_POSITION, 1708);
            success &= failed;
          }
          if (servo == 11)
          {
            const bool failed = dynamixel_->writeRegister(servo, RegisterKey::GOAL_POSITION, 1638);
            success &= failed;
          }
        }
        if (servo == 15)
        {
          const bool failed = dynamixel_->writeRegister(servo, RegisterKey::GOAL_POSITION, 2720.0);
          success &= failed;
        }
      }
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

    bool DynamixelDriver::enableDynamixels(DynamixelMode mode)
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
        //Only when using XM430 do we need Control Mode : Position or Velocity
        if (servo == 13 || servo == 17 || servo == 19 || servo == 21)
        {
          LOG_DEBUG("SETTING CONTROL MODE FOR XM430's ");
          dynamixel_->setControlMode(servo, mode);
          dynamixel_->writeRegister(servo, RegisterKey::TORQUE_ENABLE, 0);
          dynamixel_->writeRegister(servo, RegisterKey::CW_ANGLE_LIMIT, 0);
          dynamixel_->writeRegister(servo, RegisterKey::CCW_ANGLE_LIMIT, 0);

          dynamixel_->writeRegister(servo, RegisterKey::TORQUE_LIMIT,
                                    static_cast<int>(torqueLimit * kMaxTorqueLimit));

          // setting a moving speed can enable torque, so it should be the last step before
          // we manually enable torque
          dynamixel_->writeRegister(servo, RegisterKey::MOVING_SPEED, 0);
          dynamixel_->writeRegister(servo, RegisterKey::TORQUE_ENABLE, 1);
        }
        else if (servo == 15)
        {
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
            };
            LOG_DEBUG("Updating Speed of Servo MX64  to %f", servo_speed);
            dynamixel_->writeRegister(servo, RegisterKey::MOVING_SPEED, dynamixel_->getAngularSpeedToTicks(servo_speed));
          }
          dynamixel_->writeRegister(servo, RegisterKey::TORQUE_ENABLE, 1);
          LOG_DEBUG("Out Of Enabling MX64");
        }
        else if (servo == 9 || servo == 11)
        {
          LOG_DEBUG("Enabling Dynamixel for MX106");
          dynamixel_->writeRegister(servo, RegisterKey::TORQUE_ENABLE, 0);
          //TEST WITH BOTH THESE SET AND UNSET

          if (servo == 9)
          {
            dynamixel_->writeRegister(servo, RegisterKey::CW_ANGLE_LIMIT, 450);
            dynamixel_->writeRegister(servo, RegisterKey::CCW_ANGLE_LIMIT, 2967);
          }
          else
          {
            dynamixel_->writeRegister(servo, RegisterKey::CW_ANGLE_LIMIT, 724);
            dynamixel_->writeRegister(servo, RegisterKey::CCW_ANGLE_LIMIT, 2552);
          }

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
            LOG_DEBUG("Updating Speed of Servo MX106 %f", servo_speed);
            dynamixel_->writeRegister(servo, RegisterKey::MOVING_SPEED, dynamixel_->getAngularSpeedToTicks(servo_speed));
          }
          dynamixel_->writeRegister(servo, RegisterKey::TORQUE_ENABLE, 1);
          LOG_DEBUG("Out Of Enabling");
        }
      }
      //call set home position function here
      if (!goHome())
      {
        LOG_DEBUG("HOME POSITION FAILED!")
      };
      return true;
    }

    bool DynamixelArmDriver::validateKinematicTree(const kinematic_tree::KinematicTree &model)
    {
      // Number of active links match number of joints
      const auto &links = model.getActiveLinks();
      if (links.size() != kNumJoints)
      {
        return false;
      }
      // Each active link only has one degree of freedom
      if (model.getMotorStateDimensions() != kNumJoints)
      {
        return false;
      }
      // Kinematic tree is valid, get the list of joint names for parsing composite message
      joint_names_.clear();
      for (const auto *link : links)
      {
        joint_names_.push_back(link->name);
      }
      return true;
    }

    void DynamixelArmDriver::initStateSchema()
    {
      std::vector<composite::Quantity> quantities;
      for (int i = 0; i < kNumJoints; i++)
      {
        quantities.push_back(
            composite::Quantity::Scalar(joint_names_[i], composite::Measure::kPosition));
        quantities.push_back(composite::Quantity::Scalar(joint_names_[i], composite::Measure::kSpeed));
      }
      state_schema_ = composite::Schema(std::move(quantities));
    }

    void DynamixelArmDriver::initCommandParser()
    {
      // const std::string end_effector_name = get_end_effector_name();
      const std::string end_effector_name = "LARM";
      command_parser_.requestSchema(composite::Schema(joint_names_, composite::Measure::kPosition));
      return;
    }

    // Parses command from Composite message
    void DynamixelArmDriver::parseCommand()
    {
      initCommandParser();

      VectorXd command(kNumJoints);
      if (!command_parser_.parse(rx_arm_command().getProto(), rx_arm_command().buffers(), command))
      {
        reportFailure("Fails to parse joint command");
        return;
      }
      //11.375
      float conv = 11.375 command_to_arm_.clear();
      command_to_arm_.reserve(servo_ids_.size());
      //Value between 0 - 4095
      LOG_DEBUG("Command 0 is %f  and goal position is %f ", static_cast<float>(RadToDeg(command(0))), static_cast<float>(RadToDeg(command(0))) * conv);
      unsigned int goal_position = static_cast<float>(RadToDeg(command(0))) * conv;
      command_to_arm_.push_back(goal_position);

      unsigned int goal_position = static_cast<float>(RadToDeg(command(1))) * conv;
      command_to_arm_.push_back(goal_position);

      unsigned int goal_position = static_cast<float>(RadToDeg(command(2))) * conv;
      command_to_arm_.push_back(goal_position);

      unsigned int goal_position = static_cast<float>(RadToDeg(command(3))) * conv;
      command_to_arm_.push_back(goal_position);

      unsigned int goal_position = static_cast<float>(RadToDeg(command(4))) * conv;
      command_to_arm_.push_back(goal_position);

      unsigned int goal_position = static_cast<float>(RadToDeg(command(5))) * conv;
      command_to_arm_.push_back(goal_position);

      unsigned int goal_position = static_cast<float>(RadToDeg(command(6))) * conv;
      command_to_arm_.push_back(goal_position);

      if (command_to_arm_.size() != servo_ids_.size())
      {
        reportFailure("Command size list do not match");
        LOG_DEBUG(" Command Array do not match Servo array size");
        return;
      }

      bool success = true;
      for (size_t i = 0; i < servo_ids_.size(); i++)
      {
        LOG_DEBUG("Servo ID %i, Command to arm %i ")
        const bool failed = dynamixel_->writeRegister(servo_ids_[i], RegisterKey::GOAL_POSITION, command_to_arm_[i]);
        success &= failed;
      }
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

      for (int i = 0; i < 7; i++)
      {
        show("Joint" + std::to_string(i + 1) + ".command", command(i));
      }
    }
    
    //Publish Command
    void DynamixelArmDriver::publishState()
    {
      // Acqtime for publishing state messages for arm and finger
      const int64_t acqtime = getTickTimestamp();
      // Allocate tensor1d to store state data
      Tensor1d state_data(2 * kNumJoints);
      int offset = 0;

      //Read State Data from Dynamixels

      for (size_t i = 0; i < servo_ids_.size(); i++)
      {
        const int servo_id = servo_ids_[i];

        const unsigned int current_position_back =
            dynamixel_->readRegister(servo_id, RegisterKey::CURRENT_POSITION);

        const int current_speed_back =
            dynamixel_->readRegister(servo_id, RegisterKey::CURRENT_SPEED);

        LOG_DEBUG("Current Position Back %i ", current_position_back);
        LOG_DEBUG("Current Speed Back %i ", current_speed_back);

        state_data(offset++) = current_position_back;
        state_data(offset++) = current_speed_back;

        // Visualize with sight
        const std::string key = "motor_" + std::to_string(i + 1);
        show(key + ".state", actual_position(0, 0, i));
      }
      auto arm_proto_builder = tx_arm_state().initProto();
      composite::WriteSchema(state_schema_, arm_proto_builder);
      ToProto(std::move(state_data), arm_proto_builder.initValues(), tx_arm_state().buffers());
      tx_arm_state().publish(acqtime);
    }
  } 
  //namespace dynamixel
} 
// namespace isaac