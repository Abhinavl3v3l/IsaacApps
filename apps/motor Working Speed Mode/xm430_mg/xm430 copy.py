from isaac import  *
import numpy as np

JSON_FILE = r"apps/samples/xm430/xm430.app.json"
'''
      {
        "source": "xm430_mg/XM430_MG/state",
        "target": "xm430/XM430/command"
      },
      {
        "source": "dynamixel/driver/state",
        "target": "xm430/XM430/command"
      }

      ,
      {
        "name":"xm430",
        "components": [
          {
            "name": "message_ledger",
            "type": "isaac::alice::MessageLedger"
          },
          {
            "name": "XM430",
            "type": "isaac::alice::PyCodelet"
          }
        ]
      }
'''
'''
    FORMAT
        State Proto  ( elementType = float64,
        sizes = [1, 1, 1],
        scanlineStride = 0,
        dataBufferIndex = 0 )
        Schema <schema for state.capnp:StateProto>
        Data []
        State_Message Tenso Type :  <class 'numpy.ndarray'>
        State_Message Tenso Value :  [[[0.95923296]]]

    My Output
        Message received with following Detials

        State Proto Pack  ( elementType = float64,
        sizes = [1, 1, 1],
        scanlineStride = 0,
        dataBufferIndex = 0 )
        Schema <schema for state.capnp:StateProto>
        Tensor Type  <class 'numpy.ndarray'>
        Tensor  [[[2.   ]]]
        __END__

    '''

class XM430_MG(Codelet):

    def start(self):
        self.tx = self.isaac_proto_tx("StateProto", "state_tx")
        self.rx = self.isaac_proto_rx("StateProto", "state_rx")
        self.tick_periodically(0.2)

    def tick(self):
        print('TICK TEST')
        speed = 3.01
        # position = 300
        tx_message = self.tx.init()
        ''' Adding to Proto'''
        tx_message.proto.pack.elementType = 3
        # print('Set Size ')
        tx_message.proto.pack.sizes = [1,1,1]
        tx_message.proto.pack.scanlineStride = 0
        tx_message.proto.pack.dataBufferIndex = 0
        tx_message.proto.schema = "StateProto"

        ''' Adding to Buffer'''
        buffer = np.empty((1,1,1), dtype=np.dtype('float64'))
        buffer[0] = speed
        # buffer[0] = position
        tx_message.buffers = [buffer]
        tx_message.tensor = buffer[0]
        self.tx.publish()

        state_msg = self.rx.message #  MessageReader 
        if state_msg is None:
            return "Nothing Received"
        
        # print('\tMessage received with following Detials')
        # print('State Proto Pack Type {}'.format(type(state_msg.proto.pack)) )
        print('State Proto Pack  {}'.format(state_msg.proto.pack) )

        # print('State Proto Element Type {}'.format(type(state_msg.proto.pack.elementType)) )
        # print('State Proto Element {}'.format(state_msg.proto.pack.elementType) ) 
        
        # print('State Proto Size Type {}'.format(type(state_msg.proto.pack.sizes)) )
        # print('State Proto Size {}'.format(state_msg.proto.pack.sizes) )
        
        # print(' SP Scan Line Stride Type {}'.format(type(state_msg.proto.pack.scanlineStride)))
        # print(' SP Scan Line Stride {}'.format(state_msg.proto.pack.scanlineStride))
        
        # print(' State Proto Buffer Index Type {}'.format(type(state_msg.proto.pack.dataBufferIndex)))
        # print(' State Proto Buffer Index {}'.format(state_msg.proto.pack.dataBufferIndex))

        # print('Schema {}'.format(state_msg.proto.schema))


        # print('Data Type {}'.format(type(state_msg.proto.data)))
        # print('Data {}'.format(state_msg.proto.data))

        # print("Buffer Type {}".format(type(state_msg.buffers)))
        # print("Buffer  {}".format(state_msg.buffers))
        
        # print("Tensor Type  {}".format(type(state_msg.tensor)))
        print("Tensor  {}".format(state_msg.tensor))
        # print('\t__END__')
def main(): 
    print('In Node XM430')
    app = Application(app_filename=JSON_FILE)
    print("JSON LOADED")
    print('Adding Nodes')
    app.nodes["xm430_mg"].add(name="XM430_MG",ctype=XM430_MG)
    print('Nodes Added\nRunning Application')
    app.run()

if __name__ == '__main__':
    main()

