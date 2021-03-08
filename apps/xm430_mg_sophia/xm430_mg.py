'''
@author:Abhinav Rana
'''
from isaac import *
import numpy as np

# Json File Path
JSON_FILE = r"apps/samples/xm430_mg_sophia/xm430_mg.app.json"

# To view the State Proto
DEBUG_MODE = True


class XM430_MG(Codelet):

    def start(self):
        '''
            Dynamixel(TX)   -----State---->         (RX) XM430
            XM430(TX)       -----command----->      (RX) Dynamixel
        '''
        self.tx = self.isaac_proto_tx("StateProto", "command")
        self.rx = self.isaac_proto_rx("StateProto", "state")
        self.tick_periodically(0.2)

        # Sight Config for Joints
        self.config.j1 = 752        # 13  - [750-3480]
        self.config.j2 = 200       # 17  - [198-3859]
        self.config.j3 = 1644      # 19  - [1643-2297]
        self.config.j4 = 1790      # 21  - [1783-2383]

    def tick(self):
        tx_message = self.tx.init()

        ''' Adding to Proto'''
        tx_message.proto.pack.elementType = 3  # Float64
        tx_message.proto.pack.sizes = [1, 1, 4]
        tx_message.proto.pack.scanlineStride = 0
        tx_message.proto.pack.dataBufferIndex = 0
        tx_message.proto.schema = "StateProto"

        print('Sophia Left Arm Mode')
        buffer = np.empty([1, 1, 4], dtype=np.dtype('float64'))
        buffer[0][0][0] = self.config.j1      # 13  - [750-3480]
        buffer[0][0][1] = self.config.j2      # 17  - [198-3859]
        buffer[0][0][2] = self.config.j3      # 19  - [1643-2297]
        buffer[0][0][3] = self.config.j4      # 21  - [1783-2383]

        '''
          Set Buffer
        '''
        tx_message.buffers = [buffer]

        # Publish
        self.tx.publish()

        state_msg = self.rx.message  # MessageReader
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
    print('In Node XM430_MG for Sophia Left Arm')
    app = Application(app_filename=JSON_FILE)
    print("JSON LOADED")
    print('Adding Nodes')
    app.nodes["xm430_mg"].add(name="XM430_MG", ctype=XM430_MG)
    print('Nodes Added\nRunning Application')
    app.run()


if __name__ == '__main__':
    main()
