
#pragma once

#include "maestroconfig.hpp"
#include <unordered_map>
#include "engine/core/logger.hpp"

namespace isaac
{
  namespace pololu
  {
    enum Error : unsigned short
    {
       OK = 0,                              //0th Bit,   All OK
       Serial_Signal_Error = 1,             //1st Bit,   Baud Rate Mismatch
       Serial_Overrun_Error = 2,            //2nd Bit,   Occurs when the UART’s internal buffer fills up.
       Serial_RX_buffer_full = 4,           //4th Bit,   Firmware’s buffer for bytes received on the RX line is full and a byte from RX has been lost as a result.
       Serial_CRC_error = 8,                //8th Bit,   CRC Mismatch.
       Serial_protocol_error = 16,          //16th Bit,  Occures when incorrect message/command received.
       Serial_timeout_error = 32,           //32th Bit,  Occures when serial timed out.
       Script_stack_error = 64,             //64th Bit,  A bug in the user script has caused the stack to overflow or underflow
       Script_call_stack_error =128,        //128th Bit, Occures when user script call stack underflow or overflow.
       Script_program_counter_error = 256   //256th Bit, Occures when user script caused program counter to go out of bounds.
    };

    class Maestro
    {

    public:
      explicit Maestro(MaestroConfig options = MaestroConfig());

      virtual ~Maestro() {}

      // Actions
      void connect();
      void disconnect();

      // Commands

      // Set the target position of a channel to a given value in 0.25 microsecond units
      void setTargetOnChannel(unsigned char channel, unsigned short target);
      //Multiple Targets/Channels
      void setMultipleTargets(const std::unordered_map<unsigned char, unsigned short> target_on_channel_map);

      // Set the speed limit of a channel in units of (0.25 microsecond)/(10ms)
      void setSpeedOnChannel(unsigned char channel, unsigned short speed);

      // Set the acceleration limit of a channel in units of (0.25 microsecond)/(10ms)/(80ms)
      void setAccelerationOnChannel(unsigned char channel, unsigned short acceleration);

      void setPwm(unsigned short on_time, unsigned short period);

      // Return the position of a channel.
      // For a servo channel, the position is the current pulse width in 0.25 microsecond units
      void getPositionOnChannel(unsigned char channel, unsigned short &position);

      // Indicate whether the servo outputs have reached their targets or are still moving.
      void getMovingState(unsigned char &state);

      // The return the error detected by the Pololu Maestro.
      void getErrors(unsigned short &error);
      void goHome();
      // Return the minimum valid channel value in 0.25 microsecond units  700 -2200 or 2800 to 8800
      static unsigned short getMinChannelValue() { return mMinChannelValue; }

      // Return the maximum valid channel value in 0.25 microsecond units
      static unsigned short getMaxChannelValue() { return mMaxChannelValue; }

    private:
      MaestroConfig options_;
      int file_descriptor_;

      void send(unsigned char *command, unsigned int command_length);
      void receive(unsigned char *response, unsigned int response_length);

      void throw_exception(const std::string &preamble);
      static const unsigned short mMinChannelValue = 3968;
      static const unsigned short mMaxChannelValue = 8000;
    };
  }
}


