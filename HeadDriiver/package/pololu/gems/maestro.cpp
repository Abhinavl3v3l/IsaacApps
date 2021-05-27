#include <stdexcept>
#include <sstream>
#include <vector>

#include <fcntl.h>
#include <unistd.h>
// #ifdef _WIN32 //If Windows
// #define O_NOCTTY 0
// #else
#include <termios.h>
// #endif
#include <cstring>
#include <cerrno>
#include <cstdlib>

#include "maestro.hpp"
namespace isaac
{
  namespace pololu
  {

    Maestro::Maestro(MaestroConfig options)
        : options_(options)
    {
      // no-op
    }

    void Maestro::connect()
    {
      // Open the Maestro's virtual COM port.
      
      file_descriptor_ = open(options_.getDeviceFile().c_str(), O_RDWR | O_NOCTTY);
      if (file_descriptor_ == -1)
      {
        std::cout << "Error On Device" << options_.getDeviceFile() << " on device number " << int(options_.getDeviceNumber()) << std::endl;
        throw_exception(options_.getDeviceFile() + ": ");
      }
      else{std::cout << "Result : " << file_descriptor_ << std::endl;
            LOG_DEBUG("Result : %i",file_descriptor_);}

      // Set virtual COM port options
      struct termios options;
      tcgetattr(file_descriptor_, &options);
      options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
      options.c_oflag &= ~(ONLCR | OCRNL);
      options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
      tcsetattr(file_descriptor_, TCSANOW, &options);
    }

    void Maestro::disconnect()
    {
      close(file_descriptor_);
    }

    void Maestro::send(unsigned char *command, unsigned int command_length)
    {
      // std::cout << " inside send" << std::endl;
      if (write(file_descriptor_, command, command_length) == -1)
      {
        LOG_DEBUG("write un-successful");
        throw_exception(options_.getDeviceFile() + ": ");
      }
      LOG_DEBUG("write successful");
      // else{std::cout << "write successful" << std::endl;}
    }

    void Maestro::receive(unsigned char *response, unsigned int response_length)
    {
      // std::cout << " inside receive" << std::endl;
      if (read(file_descriptor_, response, response_length) == -1)
      {
        LOG_DEBUG( "read un-successful");
        throw_exception(options_.getDeviceFile() + ": ");
      }
      LOG_DEBUG( "read successful");
      // else{std::cout << "read successful" << std::endl;}
    }

    void Maestro::setTargetOnChannel(unsigned char channel, unsigned short target)
    {
      std::vector<unsigned char> command;
      LOG_DEBUG("In SetTargetOnChannel");
      switch (options_.getProtocol())
      {
      case Protocol::COMPACT:
        command.push_back(0x84);
        break;
      case Protocol::POLOLU:
        command.push_back(0xAA);
        command.push_back(options_.getDeviceNumber());
        command.push_back(0x04); //set-targer command
        break;
      default:
        throw std::runtime_error("setTargetOnChannel(): not supported by our implementation of protocol MINI_SSC");
      }
      command.push_back(channel);
      command.push_back(target & 0x7F);
      command.push_back((target >> 7) & 0x7F);
      send(command.data(), command.size());
      LOG_DEBUG("Command on channel %u is %u",channel,target);
    }

    void Maestro::setMultipleTargets(const std::unordered_map<unsigned char, unsigned short> target_on_channel_map)
    {
      if (target_on_channel_map.size() > 0)
      {
        std::vector<unsigned char> command;

        switch (options_.getProtocol())
        {
        case Protocol::COMPACT:
          command.push_back(0x84);
          break;
        case Protocol::POLOLU:
          command.push_back(0xAA);
          command.push_back(options_.getDeviceNumber());
          command.push_back(0x04);
          break;
        default:
          throw std::runtime_error("setMultipleTargets(): not supported by our implementation of protocol MINI_SSC");
        }
        for (auto pair : target_on_channel_map)
        {
          command.push_back(pair.first);
          command.push_back(pair.second & 0x7F);
          command.push_back((pair.second >> 7) & 0x7F);
        }
        send(command.data(), command.size());
      }
      else
      {
        throw std::runtime_error("setMultipleTargets(): no target-channel pairs provided in the map");
      }
    }

    void Maestro::setSpeedOnChannel(unsigned char channel, unsigned short speed)
    {
      std::vector<unsigned char> command;

      switch (options_.getProtocol())
      {
      case Protocol::COMPACT:
        command.push_back(0x87);
        break;
      case Protocol::POLOLU:
        command.push_back(0xAA);
        command.push_back(options_.getDeviceNumber());
        command.push_back(0x07);
        break;
      default:
        throw std::runtime_error("setSpeedOnChannel(): not supported by protocol MINI_SSC");
      }
      command.push_back(channel);
      command.push_back(speed & 0x7F);
      command.push_back((speed >> 7) & 0x7F);
      send(command.data(), command.size());
    }

    void Maestro::setAccelerationOnChannel(unsigned char channel, unsigned short acceleration)
    {
      std::vector<unsigned char> command;

      switch (options_.getProtocol())
      {
      case Protocol::COMPACT:
        command.push_back(0x89);
        break;
      case Protocol::POLOLU:
        command.push_back(0xAA);
        command.push_back(options_.getDeviceNumber());
        command.push_back(0x09);
        break;
      default:
        throw std::runtime_error("setAccelerationOnChannel(): not supported by protocol MINI_SSC");
      }
      command.push_back(channel);
      command.push_back(acceleration & 0x7F);
      command.push_back((acceleration >> 7) & 0x7F);
      send(command.data(), command.size());
    }

    void Maestro::setPwm(unsigned short on_time, unsigned short period)
    {
      if (options_.getType() != Type::MICRO_6)
      {
        std::vector<unsigned char> command;

        switch (options_.getProtocol())
        {
        case Protocol::COMPACT:
          command.push_back(0x8A);
          break;
        case Protocol::POLOLU:
          command.push_back(0xAA);
          command.push_back(options_.getDeviceNumber());
          command.push_back(0x0A);
          break;
        default:
          throw std::runtime_error("setPwm(): not supported by protocol MINI_SSC");
        }
        command.push_back(on_time & 0x7F);
        command.push_back((on_time >> 7) & 0x7F);
        command.push_back(period & 0x7F);
        command.push_back((period >> 7) & 0x7F);
        send(command.data(), command.size());
      }
      else
      {
        throw std::runtime_error("getMovingState(): not supported on platform MICRO_6");
      }
    }

    void Maestro::getPositionOnChannel(unsigned char channel, unsigned short &position)
    {
      std::vector<unsigned char> command;
      
      switch (options_.getProtocol())
      {
      case Protocol::COMPACT:
        command.push_back(0x90);
        break;
      case Protocol::POLOLU:
        command.push_back(0xAA);
        command.push_back(options_.getDeviceNumber());
        command.push_back(0x10);
        break;
      default:
        throw std::runtime_error("getPositionOnChannel(): not supported by protocol MINI_SSC");
      }
      command.push_back(channel);

      // std::cout << "Sending Get Position On Channel " << channel << std::endl;
      // std::cout << "Sending following command to Device" << std::endl;
      // for (auto i : command){std::cout << int(i) << " ";}
      send(command.data(), command.size());

      unsigned int response_length = 2;
      unsigned char *response = (unsigned char *)std::calloc(response_length, sizeof(unsigned char));

      // std::cout << "Receiving Command " << channel << std::endl;
      receive(response, response_length);
      // std::cout << "Received Command " << channel << std::endl;
      // for (int i = 0; i < 2; i++){std::cout << " Response is " << response[i] << std::endl;}
      position = (response[1] << 8) | response[0];
      LOG_DEBUG("Position returned is %u", position);
    }

    void Maestro::getMovingState(unsigned char &state)
    {
      if (options_.getType() != Type::MICRO_6)
      {
        std::vector<unsigned char> command;

        switch (options_.getProtocol())
        {
        case Protocol::COMPACT:
          command.push_back(0x93);
          break;
        case Protocol::POLOLU:
          command.push_back(0xAA);
          command.push_back(options_.getDeviceNumber());
          command.push_back(0x13);
          break;
        default:
          throw std::runtime_error("getMovingState(): not supported by proto col MINI_SSC");
        }
        send(command.data(), command.size());

        unsigned int response_length = 1;
        unsigned char *response = (unsigned char *)std::calloc(response_length, sizeof(unsigned char));

        receive(response, response_length);

        state = response[0];
      }
      else
      {
        throw std::runtime_error("getMovingState(): not supported on platform MICRO_6");
      }
    }

    void Maestro::getErrors(unsigned short &error)
    {
      std::vector<unsigned char> command;

      switch (options_.getProtocol())
      {
      case Protocol::COMPACT:
        command.push_back(0xA1);
        break;
      case Protocol::POLOLU:
        command.push_back(0xAA);
        command.push_back(options_.getDeviceNumber());
        command.push_back(0x21);
        break;
      default:
        throw std::runtime_error("getErrors(): not supported by protocol MINI_SSC");
      }
      send(command.data(), command.size());

      unsigned int response_length = 2;
      unsigned char *response = (unsigned char *)std::calloc(response_length, sizeof(unsigned char));

      receive(response, response_length);

      error = (response[1] << 8) | response[0];
    }

    void Maestro::goHome()
    {
      std::cout << "Inside goHome" << std::endl;

      std::vector<unsigned char> command;

      switch (options_.getProtocol())
      {
      case Protocol::COMPACT:
        command.push_back(0xA2);
        break;
      case Protocol::POLOLU:
        command.push_back(0xAA);
        command.push_back(options_.getDeviceNumber());
        command.push_back(0x22);
        break;
      default:
        throw std::runtime_error("goHome(): not supported by protocol MINI_SSC");
      }
      send(command.data(), command.size());
    }

    void Maestro::throw_exception(const std::string &preamble)
    {
      std::stringstream error_msg;
      error_msg << preamble << std::strerror(errno);
      throw std::runtime_error(error_msg.str());
    }

  }
}
