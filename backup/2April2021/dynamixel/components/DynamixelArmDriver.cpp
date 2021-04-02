#include "DynamixelArmDriver.hpp"
#include <string>
#include <utility>
#include <vector>

#include "engine/core/assert.hpp"
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
      // const int servos = {9, 11, 15, 13, 17, 19, 21};
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

      // Initialize Dynamixel motors for each type specifying their models
      dynamixel_106.reset(
          new Dynamixel(get_port().c_str(), get_baudrate(), Model::MX106));
      dynamixel_430.reset(
          new Dynamixel(get_port().c_str(), get_baudrate(), Model::XM430));
      dynamixel_64.reset(
          new Dynamixel(get_port().c_str(), get_baudrate(), Model::MX64));

      if (!initializeServoIds())
        return;

      if (!enableDynamixels(get_control_mode()))
        return;

      initStateSchema();
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
    void DynamixelArmDriver::tick()
    {
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
        else
        {
          LOG_DEBUG("COMMAND TIME INCONSISTENCY");
        }
      }
      publishState();
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
        LOG_DEBUG("SERVO %d added", servo_id);
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
        //XM430

        LOG_DEBUG(" SERVO ID IS %i", servo);
        if (servo == 13 || servo == 17 || servo == 19 || servo == 21)
        {
          if (servo == 13)
          {
            // //Checks made to see if torque is enabled
            // if(dynamixel_430->readRegister(servo, RegisterKey::TORQUE_ENABLE) == 1){
            // LOG_DEBUG("TORQUE IS ENABLED WHILE INITILIZING");
            // }
            // else{
            //   LOG_DEBUG("TORQUE IS DISABLED WHILE INITIALIZING");
            // }
            const bool failed = dynamixel_430->writeRegister(servo, RegisterKey::GOAL_POSITION, 2000);
            success &= failed;
          }
          else if (servo == 17)
          {
            const bool failed = dynamixel_430->writeRegister(servo, RegisterKey::GOAL_POSITION, 2000);
            success &= failed;
          }
          else if (servo == 19)
          {
            const bool failed = dynamixel_430->writeRegister(servo, RegisterKey::GOAL_POSITION, 2000);
            success &= failed;
          }
          else if (servo == 21)
          {
            const bool failed = dynamixel_430->writeRegister(servo, RegisterKey::GOAL_POSITION, 2000);
            success &= failed;
          }
        }
        //mx106
        if (servo == 9 || servo == 11)
        {
          if (servo == 9)
          {
            const bool failed = dynamixel_106->writeRegister(servo, RegisterKey::GOAL_POSITION, 2300);
            success &= failed;
          }
          else if (servo == 11)
          {
            const bool failed = dynamixel_106->writeRegister(servo, RegisterKey::GOAL_POSITION, 820);
            success &= failed;
          }
        }
        //mx64
        if (servo == 15)
        {
          const bool failed = dynamixel_64->writeRegister(servo, RegisterKey::GOAL_POSITION, 900);
          success &= failed;
        }
      }
      if (!success)
      {
        const byte status_106 = dynamixel_106->getStatus();
        const byte status_430 = dynamixel_430->getStatus();
        const byte status_64 = dynamixel_64->getStatus();
        const bool serious_error_106 =
            (status_106 & kStatusOverheating) || (status_106 & kStatusOverloaded);
        const bool serious_error_430 =
            (status_430 & kStatusOverheating) || (status_430 & kStatusOverloaded);
        const bool serious_error_64 =
            (status_64 & kStatusOverheating) || (status_64 & kStatusOverloaded);

        if (serious_error_106)
        {
          reportFailure("Dynamixel_106 serious error detected; disabling servos");
          return false;
        }

        if (serious_error_430)
        {
          reportFailure("Dynamixel_430 serious error detected; disabling servos");
          return false;
        }

        if (serious_error_64)
        {
          reportFailure("Dynamixel_64 serious error detected; disabling servos");
          return false;
        }
        // do not log other errors, since the dynamixel API has already logged the error
      }

      return true;
    }

    bool DynamixelArmDriver::enableDynamixels(DynamixelMode mode)
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
          LOG_DEBUG("SETTING CONTROL MODE FOR XM430's");
          dynamixel_430->setControlMode(servo, mode);
          dynamixel_430->writeRegister(servo, RegisterKey::TORQUE_ENABLE, 0);
          dynamixel_430->writeRegister(servo, RegisterKey::CW_ANGLE_LIMIT, 0);
          dynamixel_430->writeRegister(servo, RegisterKey::CCW_ANGLE_LIMIT, 0);

          dynamixel_430->writeRegister(servo, RegisterKey::TORQUE_LIMIT,
                                       static_cast<int>(torqueLimit * kMaxTorqueLimit));

          // setting a moving speed can enable torque, so it should be the last step before
          // we manually enable torque
          dynamixel_430->writeRegister(servo, RegisterKey::MOVING_SPEED, 0);
          dynamixel_430->writeRegister(servo, RegisterKey::TORQUE_ENABLE, 1);
        }
        //MX64
        else if (servo == 15)
        {
          LOG_DEBUG("Enabling Dynamixel for MX64");
          dynamixel_64->writeRegister(servo, RegisterKey::TORQUE_ENABLE, 0);
          dynamixel_64->writeRegister(servo, RegisterKey::CW_ANGLE_LIMIT, 729);
          dynamixel_64->writeRegister(servo, RegisterKey::CCW_ANGLE_LIMIT, 3606);

          dynamixel_64->writeRegister(servo, RegisterKey::TORQUE_LIMIT,
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
            dynamixel_64->writeRegister(servo, RegisterKey::MOVING_SPEED, dynamixel_64->getAngularSpeedToTicks(servo_speed));
          }
          dynamixel_64->writeRegister(servo, RegisterKey::TORQUE_ENABLE, 1);
          LOG_DEBUG("Out Of Enabling MX64");
        }
        //MX106
        else if (servo == 9 || servo == 11)
        {
          LOG_DEBUG("Enabling Dynamixel for MX106");
          dynamixel_106->writeRegister(servo, RegisterKey::TORQUE_ENABLE, 0);
          //TEST WITH BOTH THESE SET AND UNSET

          if (servo == 9)
          {
            dynamixel_106->writeRegister(servo, RegisterKey::CW_ANGLE_LIMIT, 450);
            dynamixel_106->writeRegister(servo, RegisterKey::CCW_ANGLE_LIMIT, 2967);
          }
          else
          {
            dynamixel_106->writeRegister(servo, RegisterKey::CW_ANGLE_LIMIT, 724);
            dynamixel_106->writeRegister(servo, RegisterKey::CCW_ANGLE_LIMIT, 2552);
          }

          dynamixel_106->writeRegister(servo, RegisterKey::TORQUE_LIMIT,
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
            dynamixel_106->writeRegister(servo, RegisterKey::MOVING_SPEED, dynamixel_106->getAngularSpeedToTicks(servo_speed));
          }
          dynamixel_106->writeRegister(servo, RegisterKey::TORQUE_ENABLE, 1);
          LOG_DEBUG("Out Of Enabling");
        }
      }
      if (get_gohome())
      {
        if (!goHome())
        {
          LOG_DEBUG("HOME POSITION FAILED!");
        }
      }
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
      LOG_DEBUG("parseCommand START");
      initCommandParser();

      VectorXd command(kNumJoints);
      if (!command_parser_.parse(rx_arm_command().getProto(), rx_arm_command().buffers(), command))
      {
        reportFailure("Fails to parse joint command");
        return;
      }
      // float conv = 11.375;
      command_to_arm_.clear();
      command_to_arm_.reserve(servo_ids_.size());
      //Value between 0 - 4095
      // LOG_DEBUG("Command 0 in radians is  %f  and goal position is %f  dynamixel pos %f", command(0), static_cast<float>(RadToDeg(command(0))), (static_cast<float>(RadToDeg(command(0))) * 11.375) );
      // LOG_DEBUG("Command 1 in radians is  %f  and goal position is %f ", command(1), static_cast<float>(RadToDeg(command(1))) );
      // LOG_DEBUG("Command 2 in radians is  %f  and goal position is %f ", command(2), static_cast<float>(RadToDeg(command(2))) );
      // LOG_DEBUG("Command 3 in radians is %f  and goal position is %f ", command(3), static_cast<float>(RadToDeg(command(3))) );
      // LOG_DEBUG("Command 4 in radians is %f  and goal position is %f ", command(4), static_cast<float>(RadToDeg(command(4))) );
      // LOG_DEBUG("Command 5 in radians is %f  and goal position is %f ", command(5), static_cast<float>(RadToDeg(command(5))) );
      // LOG_DEBUG("Command 6 in radians is %f  and goal position is %f ", command(6), static_cast<float>(RadToDeg(command(6))) );

      unsigned int goal_position = (static_cast<float>(RadToDeg(command(0))) * 11.375);
      command_to_arm_.push_back(goal_position);

      unsigned int goal_position2 = (static_cast<float>(RadToDeg(command(1))) * 11.375);
      command_to_arm_.push_back(goal_position2);

      unsigned int goal_position3 = (static_cast<float>(RadToDeg(command(2))) * 11.375);
      command_to_arm_.push_back(goal_position3);

      unsigned int goal_position4 = (static_cast<float>(RadToDeg(command(3))) * 11.375);
      command_to_arm_.push_back(goal_position4);

      unsigned int goal_position5 = (static_cast<float>(RadToDeg(command(4))) * 11.375);
      command_to_arm_.push_back(goal_position5);

      unsigned int goal_position6 = (static_cast<float>(RadToDeg(command(5))) * 11.375);
      command_to_arm_.push_back(goal_position6);

      unsigned int goal_position7 = (static_cast<float>(RadToDeg(command(6))) * 11.375);
      command_to_arm_.push_back(goal_position7);

      if (command_to_arm_.size() != servo_ids_.size())
      {
        reportFailure("Command size list do not match");
        LOG_DEBUG(" Command Array do not match Servo array size");
        return;
      }

      bool success = true;
      for (size_t i = 0; i < servo_ids_.size(); i++)
      {
        if (servo_ids_[i] == 13 || servo_ids_[i] == 17 || servo_ids_[i] == 19 || servo_ids_[i] == 21)
        {
          LOG_DEBUG("Servo ID %i, Command to arm %i ", servo_ids_[i], command_to_arm_[i]);
          const bool failed = dynamixel_430->writeRegister(servo_ids_[i], RegisterKey::GOAL_POSITION, command_to_arm_[i]);
          success &= failed;
        }
        else if (servo_ids_[i] == 9 || servo_ids_[i] == 11)
        {
          LOG_DEBUG("Servo ID %i, Command to arm %i ", servo_ids_[i], command_to_arm_[i]);
          const bool failed = dynamixel_106->writeRegister(servo_ids_[i], RegisterKey::GOAL_POSITION, command_to_arm_[i]);
          success &= failed;
        }
        else if (servo_ids_[i] == 15)
        {
          LOG_DEBUG("Servo ID %i, Command to arm %i ", servo_ids_[i], command_to_arm_[i]);
          const bool failed = dynamixel_64->writeRegister(servo_ids_[i], RegisterKey::GOAL_POSITION, command_to_arm_[i]);
          success &= failed;
        }
        else
        {
          LOG_ERROR("WRONG SERVO ID FOUND ");
        }
      }
      if (!success)
      {
        const byte status_106 = dynamixel_106->getStatus();
        const byte status_430 = dynamixel_430->getStatus();
        const byte status_64 = dynamixel_64->getStatus();
        const bool serious_error_106 =
            (status_106 & kStatusOverheating) || (status_106 & kStatusOverloaded);
        const bool serious_error_430 =
            (status_430 & kStatusOverheating) || (status_430 & kStatusOverloaded);
        const bool serious_error_64 =
            (status_64 & kStatusOverheating) || (status_64 & kStatusOverloaded);

        if (serious_error_106)
        {
          reportFailure("Dynamixel_106 serious error detected; disabling servos");
          return;
        }

        if (serious_error_430)
        {
          reportFailure("Dynamixel_430 serious error detected; disabling servos");
          return;
        }

        if (serious_error_64)
        {
          reportFailure("Dynamixel_64 serious error detected; disabling servos");
          return;
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
      LOG_DEBUG("INSIDE PUBLISH STATE ");
      // Acqtime for publishing state messages for arm and finger
      const int64_t acqtime = getTickTimestamp();
      // Allocate tensor1d to store state data
      Tensor1d state_data(2 * kNumJoints);
      int offset = 0;
      float speed = 1.0f;
      //Read State Data from Dynamixels
      for (size_t i = 0; i < servo_ids_.size(); i++)
      {
        const int servo_id = servo_ids_[i];
        if (servo_id == 13 || servo_id == 17 || servo_id == 19 || servo_id == 21)
        {
          unsigned int current_position_back =
              dynamixel_430->readRegister(servo_id, RegisterKey::CURRENT_POSITION);

          float pos = static_cast<float>(DegToRad(current_position_back / 11.3778f));
          // const int current_speed_back =
          // dynamixel_430->readRegister(servo_id, RegisterKey::CURRENT_SPEED);

          LOG_DEBUG("Current Position Back %i converted position is  %f of servo id %i ", current_position_back, pos, servo_id);
          // LOG_DEBUG("Current Speed Back %i ", speed);

          state_data(offset++) = pos;
          state_data(offset++) = speed;
          // Visualize with sight
          const std::string key_pos = "pos_motor_" + std::to_string(i + 1);
          show(key_pos + ".state", pos);
          const std::string key_speed = "speed_motor_" + std::to_string(i + 1);
          show(key_speed + ".state", speed);
        }
        if (servo_id == 9 || servo_id == 11)
        {
          unsigned int current_position_back =
              dynamixel_106->readRegister(servo_id, RegisterKey::CURRENT_POSITION);
          float pos = static_cast<float>(DegToRad(current_position_back / 11.3778f));
          // const int current_speed_back =
          //     dynamixel_106->readRegister(servo_id, RegisterKey::CURRENT_SPEED);

          LOG_DEBUG("Current Position Back %i converted position is  %f of servo id %i ", current_position_back, pos, servo_id);
          // LOG_DEBUG("Current Speed Back %i ", speed);

          state_data(offset++) = pos;
          state_data(offset++) = speed;
          // Visualize with sight
          const std::string key_pos = "pos_motor_" + std::to_string(i + 1);
          show(key_pos + ".state", pos);
          const std::string key_speed = "speed_motor_" + std::to_string(i + 1);
          show(key_speed + ".state", speed);
        }

        if (servo_id == 15)
        {
          unsigned int current_position_back =
              dynamixel_64->readRegister(servo_id, RegisterKey::CURRENT_POSITION);
          LOG_DEBUG("Before Conversion Current Position Back %i of servo id %i ", current_position_back, servo_id);
          float pos = static_cast<float>(DegToRad(current_position_back / 11.3778f));
          // const int current_speed_back =
          //     dynamixel_64->readRegister(servo_id, RegisterKey::CURRENT_SPEED);

          LOG_DEBUG("Current Position Back %i converted position is  %f of servo id %i ", current_position_back, pos, servo_id);
          // LOG_DEBUG("Current Speed Back %i ", speed);

          state_data(offset++) = pos;
          state_data(offset++) = speed;
          // Visualize with sight
          const std::string key_pos = "pos_motor_" + std::to_string(i + 1);
          show(key_pos + ".state", pos);
          const std::string key_speed = "speed_motor_" + std::to_string(i + 1);
          show(key_speed + ".state", speed);
        }
      }
      auto arm_proto_builder = tx_arm_state().initProto();
      composite::WriteSchema(state_schema_, arm_proto_builder);
      ToProto(std::move(state_data), arm_proto_builder.initValues(), tx_arm_state().buffers());
      tx_arm_state().publish(acqtime);
    }

    void DynamixelArmDriver::disableDynamixels()
    {
      for (const uint8_t servo : servo_ids_)
      {
        // disabling torque needs to be after setting the moving speed, otherwise the servos will
        // automatically enable torque(
        LOG_DEBUG("Disabling Servo ID %i", servo);
        if (servo == 13 || servo == 17 || servo == 19 || servo == 21)
        {
          dynamixel_430->writeRegister(servo, RegisterKey::MOVING_SPEED, 0);
          if (dynamixel_430->readRegister(servo, RegisterKey::TORQUE_ENABLE) == 1)
          {
            LOG_DEBUG("TORQUE IS ENABLED");
          }
          dynamixel_430->writeRegister(servo, RegisterKey::TORQUE_ENABLE, 0);
        }
        if (servo == 9 || servo == 11)
        {
          dynamixel_106->writeRegister(servo, RegisterKey::MOVING_SPEED, 0);
          dynamixel_106->writeRegister(servo, RegisterKey::TORQUE_ENABLE, 0);
        }

        if (servo == 15)
        {
          dynamixel_64->writeRegister(servo, RegisterKey::MOVING_SPEED, 0);
          dynamixel_64->writeRegister(servo, RegisterKey::TORQUE_ENABLE, 1);
          dynamixel_64->writeRegister(servo, RegisterKey::TORQUE_ENABLE, 0);
          if (dynamixel_64->readRegister(servo, RegisterKey::TORQUE_ENABLE) == 1)
          {
            LOG_DEBUG("TORQUE IS ENABLED for MX64 DID NOT REFLECT");
          }
        }
      }
    }
  }
  //namespace dynamixel
}
// namespace isaac