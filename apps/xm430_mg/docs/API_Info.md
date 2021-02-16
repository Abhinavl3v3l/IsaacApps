

### isaac.dynamixel.DynamixelDriver

Description

A motor driver for a Daisy chain of Dynamixel motors. This codelet receives desired motor speed commands via a message  and publishes the current motor speeds as a state proto.Multiple checks and safe guards are used to protected the driver from misuse. The codelet also currently features a small debug mode in which individual motors can be tested  with constant speed.



## isaac.PanTiltDriver

**Description**

> The PanTiltDriver class is a driver for a pan/tilt unit based on  Dynamixel motors.  As its name indicates, the unit is composed of two  joints, the first one performing a rotation  along the z axis (pan), the second performing a rotation along the y axis (tilt). Each joint have  a name (for example ‘pan’ and ‘tilt’), and this driver will in addition  to controlling the  pan/tilt updates the PoseTree: the two  transformations corresponding to each joint will be  updated. The name  of the edge in the PoseTree are: pan_in_T_pan_out and tilt_in_T_tilt_out where  “pan” and “tilt” are the name of both joint.  In order to make  these transform useful you should add to the PoseTree (for example using a  PoseInitializer codelet) the transformations from the robot to the  pan_in and from the pan joint  to the tilt joint (pan_out_T_tilt_in).

**Type:** Codelet - This component ticks either periodically or when it receives messages.

**Incoming messages**

- command [[StateProto](https://docs.nvidia.com/isaac/isaac/doc/doc/message_api.html#stateproto)]:  Current command for pan/tilt unit

**Outgoing messages**

- state [[StateProto](https://docs.nvidia.com/isaac/isaac/doc/doc/message_api.html#stateproto)]:  The state of the pan tilt unit
- motors [[DynamixelMotorsProto](https://docs.nvidia.com/isaac/isaac/doc/doc/message_api.html#dynamixelmotorsproto)]:  State of Dynamixel motors

**Parameters**

- use_speed_control [*bool*] [default=true]:  If set to true dynamixels are controlled in speed mode, otherwise they are controlled in  position mode
- usb_port [*string*] [default=”/dev/ttyUSB0”]:  USB port used to connect to the bus (A2D2 USB adapter)
- pan_servo_id [*int*] [default=1]:  Dynamixel ID for pan servo
- tilt_servo_id [*int*] [default=2]:  Dynamixel ID for tilt servo
- tilt_min [*double*] [default=-0.5]:  Minimum value valid for tilt
- tilt_max [*double*] [default=2.0]:  Maximum value valid for tilt
- pan_min [*double*] [default=-Pi<double>]:  Minimum value valid for pan
- pan_max [*double*] [default=Pi<double>]:  Maximum value valid for pan
- pan_offset [*double*] [default=5.83]:  Constant offset in the pan angle such as pan = 0 has the end effector looking forward (X axis)
- tilt_offset [*double*] [default=4.73]:  Constant offset in the tilt angle such as tilt = 0 has the end effector looking horizontally
- baudrate [*int*]  [default=static_cast<int>(dynamixel::Baudrate::k1M)]:  Baudrate of the Dynamixel bus. See packages/dynamixel/gems/registers.hpp for  options.  TODO Remove when refactored to use DynamixelDriver class
- model [*int*]  [default=static_cast<int>(dynamixel::Model::XM430)]:  What kind of dynamixel model it is: (AX12A = 0, XM430 = 1, MX12W = 2)   TODO(jberling) refactor pan tilt to use DynamixelDriver class and switch to enum
- pan_joint_frame [*string*] [default=”pan”]:  Name of the pan joint frame. The edge pan_in_T_pan_out will be added to the PoseTree.
- tilt_joint_frame [*string*] [default=”tilt”]:  Name of the tilt joint frame. The edge tilt_in_T_tilt_out will be added to the PoseTree.



------

## Message Api

dynamixel_motors > DynamixelMotorsProto

```c++
#Commands commands for a set of dynamixel motors running for example as a dasy chain.

struct DynamixelMotorsProto {
  // Motor protos
  struct MotorProto {
    // Motor ID
    id @0: Int64;
    // Current position
    position @1: Int16;
    // Tick in milliseconds since startup
    tick @2: Int64;
    // Is the servo moving
    moving @3: Bool;
  }
  // A single command per motor
  motors @0: List(MotorProto);
}
```

### StateProto

```c++
// A message used to transport states of dynamic systems. This is used closely together with the state gem to be found in //engine/gems/state. For example you can define the robot state using the compile-time types defined in that gem, and then serialize that state into a message. The receiver of that message can deserialize the state back into the compile-time robot state type.
struct StateProto {
  // A densely packed representation of the state data as floating point numbers. The lowest rank
  // indicates state element dimension, the second rank is time for a potential timeseries, and
  // the third rank is for the batch index.
  pack @0: TensorProto;
  // The schema describing the format of the state vector
  schema @1: Text;
  // Alternative way to pass the data (for python)
  data @2: List(Float64);
}
```

### TensorProto

```c++
//A n-dimensional tensor
struct TensorProto {
  // Type of channel elements
  elementType @0: ElementType;
  // Dimensions of the tensor
  sizes @1: List(Int32);
  // deprecated - not used anymore
  scanlineStride @2: UInt32;
  // Index of buffer which contains the tensor data
  dataBufferIndex @3: UInt16;
}
```