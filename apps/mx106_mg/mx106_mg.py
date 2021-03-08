'''
@author:Abhinav Rana
'''
from isaac import *
import numpy as np

# Json File Path
JSON_FILE = r"apps/samples/mx106_mg/mx106_mg.app.json"

# To view the State Proto
DEBUG_MODE = True

'''
Config 

      "driver64": {
        "tick_period": "50Hz",
        "port": "/dev/ttyUSB0",
        "servo_ids": [
          15
        ],
        "servo_model": "MX64",
        "baudrate": "k1M",
        "max_speed":4.8,
        "speed":1.0
      },
      "driver430": {
        "tick_period": "50Hz",
        "port": "/dev/ttyUSB0",
        "servo_ids": [
          13, 17, 19, 21
        ],
        "servo_model": "XM430",
        "baudrate": "k1M",
        "debug_mode": false,
        "debug_speed": 1.0,
        "control_mode": "position",
        "max_speed": 3.0,
        "debug_position_mode": true,
        "position": 2000,
        "speed" : 1.0 
      },

Components

          {
            "name": "driver106",
            "type": "isaac::dynamixel::DynamixelDriver"
          },
          {
            "name": "driver430",
            "type": "isaac::dynamixel::DynamixelDriver"
          },

Edges
          {
        "source": "mx64_mg/MX64_MG/state",
        "target": "dynamixel/driver64/command"
      },
      {
        "source": "dynamixel/driver64/state",
        "target": "mx64_mg/MX64_MG/command"
      },
      {
        "source": "mx106_mg/MX106_MG/state",
        "target": "dynamixel/driver106/command"
      },
      {
        "source": "dynamixel/driver106/state",
        "target": "mx106_mg/MX106_MG/command"
      },{
        "source": "xm430_mg/XM430_MG/state",
        "target": "dynamixel/driver430/command"
      },
      {
        "source": "dynamixel/driver430/state",
        "target": "xm430_mg/XM430_MG/command"
      }
'''


class MX106_MG(Codelet):

    def start(self):
        # MX106
        self.tx = self.isaac_proto_tx("StateProto", "command")
        self.rx = self.isaac_proto_rx("StateProto", "state")
        self.tick_periodically(0.2)

        # Sight Config for Joints
        self.config.mx106_1 = 1500  # 8  (471 - 2848)
        self.config.mx106_2 = 1000  # 10 (758 - 2585)

    def tick(self):
        '''TX and RX between  MX106 and Dynamixel Drivers'''
        tx_message = self.tx.init()

        ''' Adding to Proto'''
        tx_message.proto.pack.elementType = 3  # Float64
        tx_message.proto.pack.sizes = [1, 1, 2]
        tx_message.proto.pack.scanlineStride = 0
        tx_message.proto.pack.dataBufferIndex = 0
        tx_message.proto.schema = "StateProto"

        buffer_106 = np.empty([1, 1, 2], dtype=np.dtype('float64'))
        buffer_106[0][0][0] = self.config.mx106_1
        buffer_106[0][0][1] = self.config.mx106_2

        # Set Buffer for MX106
        tx_message.buffers = [buffer_106]

        # Publish
        self.tx.publish()

        state_msg = self.rx.message  # MessageReader
        if state_msg is None:
            return "Nothing Received"
        if(DEBUG_MODE):
            print('\t__ MX106 State Message __')
            # print('State Proto Pack  {}'.format(state_msg.proto.pack))
            # print('Schema {}'.format(state_msg.proto.schema))
            # print('Data {}'.format(state_msg.proto.data))
            # print("Buffer  {}".format(state_msg.buffers))
            print("Tensor  {}".format(state_msg.tensor))
            print('\t__END__')
        # End of 106 Driver


def main():
    print('Driver Test for all dynamixel')
    app = Application(app_filename=JSON_FILE)
    print("JSON LOADED")
    print('Adding Nodes')
    app.nodes["mx106_mg"].add(name="MX106_MG", ctype=MX106_MG)
    print('Nodes Added\nRunning Application')
    app.run()


if __name__ == '__main__':
    main()
