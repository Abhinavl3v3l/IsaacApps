'''
@author:Abhinav Rana
'''
from isaac import *
import numpy as np

# Json File Path
JSON_FILE = r"apps/samples/mx64_mg/mx64_mg.app.json"

# To view the State Proto
DEBUG_MODE = True

class MX106_MG(Codelet):

    def start(self):
        #MX106
        self.tx_106 = self.isaac_proto_tx("StateProto", "state")
        self.rx_106 = self.isaac_proto_rx("StateProto", "command")

        
        self.tick_periodically(0.2)

        # Sight Config for Joints
        # self.config.mx64 = 2000       # 15 - [1349 - 4092]

    def tick(self):
        '''TX and RX between  MX106 and Dynamixel Drivers'''
        tx_106_message = self.tx_106.init()

        ''' Adding to Proto'''
        tx_106_message.proto.pack.elementType = 3  # Float64
        tx_106_message.proto.pack.sizes = [1, 1, 2]
        tx_106_message.proto.pack.scanlineStride = 0
        tx_106_message.proto.pack.dataBufferIndex = 0
        tx_106_message.proto.schema = "StateProto"

        
        buffer_106 = np.empty([1, 1, 2], dtype=np.dtype('float64'))
        buffer_106[0][0][0] =  2000     
        buffer_106[0][0][1] =  2333     

        #Set Buffer for MX106
        tx_106_message.buffers = [buffer_106]

        # Publish
        self.tx_106.publish()

        state_msg = self.rx_106.message  # MessageReader
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
        #End of 106 Driver


        
class MX64_MG(Codelet):

    def start(self):
        #MX64
        self.tx_64 = self.isaac_proto_tx("StateProto", "state_64")
        self.rx_64 = self.isaac_proto_rx("StateProto", "command_64")

        #MX106
        self.tx_106 = self.isaac_proto_tx("StateProto", "state_106")
        self.rx_106 = self.isaac_proto_rx("StateProto", "command_106")

        #XM430
        self.tx_430 = self.isaac_proto_tx("StateProto", "state_430")
        self.rx_430 = self.isaac_proto_rx("StateProto", "command_430")

        
        self.tick_periodically(0.2)

        # Sight Config for Joints
        # self.config.mx64 = 2000       # 15 - [1349 - 4092]

    def tick(self):
        '''TX and RX between  MX106 and Dynamixel Drivers'''
        tx_106_message = self.tx_106.init()

        ''' Adding to Proto'''
        tx_106_message.proto.pack.elementType = 3  # Float64
        tx_106_message.proto.pack.sizes = [1, 1, 2]
        tx_106_message.proto.pack.scanlineStride = 0
        tx_106_message.proto.pack.dataBufferIndex = 0
        tx_106_message.proto.schema = "StateProto"

        
        buffer_106 = np.empty([1, 1, 2], dtype=np.dtype('float64'))
        buffer_106[0][0][0] =  2000     
        buffer_106[0][0][1] =  2333     

        #Set Buffer for MX106
        tx_106_message.buffers = [buffer_106]

        # Publish
        self.tx_106.publish()

        state_msg = self.rx_106.message  # MessageReader
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
        #End of 106 Driver


        '''
            TX and RX between  MX64 and Dynamixel Drivers
        '''
        tx_64_message = self.tx_64.init()

        ''' Adding to Proto'''
        tx_64_message.proto.pack.elementType = 3  # Float64
        tx_64_message.proto.pack.sizes = [1, 1, 1]
        tx_64_message.proto.pack.scanlineStride = 0
        tx_64_message.proto.pack.dataBufferIndex = 0
        tx_64_message.proto.schema = "StateProto"

        
        buffer_64 = np.empty([1, 1, 1], dtype=np.dtype('float64'))
        buffer_64[0][0][0] =  2000     #8

        #Set Buffer for MX106
        tx_64_message.buffers = [buffer_64]
        
        # Publish
        self.tx_106.publish()

        state_msg = self.rx_106.message  # MessageReader
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
        #End of 106 Driver



        '''TX and RX between  XM430 and Dynamixel Drivers'''
        tx_430_message = self.tx_430.init()

        ''' Adding to Proto'''
        tx_430_message.proto.pack.elementType = 3  # Float64
        tx_430_message.proto.pack.sizes = [1, 1, 3]
        tx_430_message.proto.pack.scanlineStride = 0
        tx_430_message.proto.pack.dataBufferIndex = 0
        tx_430_message.proto.schema = "StateProto"

        # Allocate Buffer
        buffer_430 = np.empty([1, 1, 3], dtype=np.dtype('float64'))
        buffer_430[0][0][0] =  2000         
        buffer_430[0][0][1] =  2333         
        buffer_430[0][0][2] =  1024         

        '''
          Set Buffer
        '''
        tx_430_message.buffers = [buffer_430]

        # Publish
        self.tx_430.publish()

        state_msg = self.rx_430.message  # MessageReader
        if state_msg is None:
            return "Nothing Received"
        if(DEBUG_MODE):
            print('\t__Message received with following Detials__')
            print('State Proto Pack  {}'.format(state_msg.proto.pack))
            print('Schema {}'.format(state_msg.proto.schema))
            print('Data {}'.format(state_msg.proto.data))
            # print("Buffer  {}".format(state_msg.buffers))
            print("Tensor  {}".format(state_msg.tensor))
            print('\t__END__')

class XM430_MG(Codelet):

    def start(self):

        #XM430
        self.tx_430 = self.isaac_proto_tx("StateProto", "state")
        self.rx_430 = self.isaac_proto_rx("StateProto", "command")

        
        self.tick_periodically(0.2)

        # Sight Config for Joints
        # self.config.mx64 = 2000       # 15 - [1349 - 4092]

    def tick(self):
        '''TX and RX between  XM430 and Dynamixel Drivers'''
        tx_430_message = self.tx_430.init()

        ''' Adding to Proto'''
        tx_430_message.proto.pack.elementType = 3  # Float64
        tx_430_message.proto.pack.sizes = [1, 1, 3]
        tx_430_message.proto.pack.scanlineStride = 0
        tx_430_message.proto.pack.dataBufferIndex = 0
        tx_430_message.proto.schema = "StateProto"

        # Allocate Buffer
        buffer_430 = np.empty([1, 1, 3], dtype=np.dtype('float64'))
        buffer_430[0][0][0] =  2000         
        buffer_430[0][0][1] =  2333         
        buffer_430[0][0][2] =  1024         

        '''
          Set Buffer
        '''
        tx_430_message.buffers = [buffer_430]

        # Publish
        self.tx_430.publish()

        state_msg = self.rx_430.message  # MessageReader
        if state_msg is None:
            return "Nothing Received"
        if(DEBUG_MODE):
            print('\t__Message received with following Detials__')
            print('State Proto Pack  {}'.format(state_msg.proto.pack))
            print('Schema {}'.format(state_msg.proto.schema))
            print('Data {}'.format(state_msg.proto.data))
            # print("Buffer  {}".format(state_msg.buffers))
            print("Tensor  {}".format(state_msg.tensor))
            print('\t__END__')


def main():
    print('Driver Test for all dynamixel')
    app = Application(app_filename=JSON_FILE)
    print("JSON LOADED")
    print('Adding Nodes')
    app.nodes["mx64_mg"].add(name="MX64_MG", ctype=MX64_MG)
    print('Nodes Added\nRunning Application')
    app.run()


if __name__ == '__main__':
    main()
