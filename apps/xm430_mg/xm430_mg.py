from isaac import  *
import numpy as np

JSON_FILE = r"apps/samples/xm430_mg/xm430_mg.app.json"
DEBUG_MODE = True
SOPHIA = False

'''
    FORMAT
  State Proto  ( elementType = float64,
    sizes = [1, 1, 1],
    scanlineStride = 0,
    dataBufferIndex = 0 )
  State Proto Element Type <class 'capnp.lib.capnp._DynamicEnum'>
  State Proto Element float64
  State Proto Size Type <class 'capnp.lib.capnp._DynamicListReader'>
  State Proto Size [1, 1, 1]
  SP Scan Line Stride Type <class 'int'>
  SP Scan Line Stride 0
  State Proto Buffer Index Type <class 'int'>
  State Proto Buffer Index 0
  Schema <schema for state.capnp:StateProto>
  Buffer Type <class 'list'>
  Buffer  [<memory at 0x7fabf56aa948>]
  Data Type <class 'capnp.lib.capnp._DynamicListReader'>
  Data []
  State_Message Tenso Type :  <class 'numpy.ndarray'>
  State_Message Tenso Value :  [[[0.95923296]]]


    My Output
  State Proto Pack Type <class 'capnp.lib.capnp._DynamicStructReader'>
  State Proto Pack  ( elementType = float64,
    sizes = [1, 1, 1],
    scanlineStride = 0,
    dataBufferIndex = 0 )
  State Proto Element Type <class 'capnp.lib.capnp._DynamicEnum'>
  State Proto Element float64
  State Proto Size Type <class 'capnp.lib.capnp._DynamicListReader'>
  State Proto Size [1, 1, 1]
  SP Scan Line Stride Type <class 'int'>
  SP Scan Line Stride 0
  State Proto Buffer Index Type <class 'int'>
  State Proto Buffer Index 0
  Schema <schema for state.capnp:StateProto>
  Data Type <class 'capnp.lib.capnp._DynamicListReader'>
  Data []
  Buffer Type <class 'list'>
  Buffer  [<memory at 0x7f3c547e2948>]
  Tensor Type  <class 'numpy.ndarray'>
  Tensor  [[[0.95923296]]]
    '''

class XM430_MG(Codelet):

    def start(self):
        self.tx = self.isaac_proto_tx("StateProto", "state")  
        self.rx = self.isaac_proto_rx("StateProto", "command")

        self.tick_periodically(0.2)
        # self.tick_on_message(self.rx)
        
        # self.config.speed = 2.0
        self.config.position = 2048 #0 - 4095 and should be declared to buffer element.
        

    def tick(self):
        tx_message = self.tx.init()

        ''' Adding to Proto'''
        tx_message.proto.pack.elementType = 3 #Float64
        tx_message.proto.pack.sizes = [1,1,1]
        tx_message.proto.pack.scanlineStride = 0
        tx_message.proto.pack.dataBufferIndex = 0
        tx_message.proto.schema = "StateProto"

        print('DEBUG DYNAMIXEL SINGLE ACTUATOR MODE')
        buffer = np.empty([1,1,1], dtype=np.dtype('float64')  ) 
        buffer[0][0][0] = self.config.position 
        tx_message.buffers = [buffer]
        # buffer[0][0][0] = self.config.speed
        # tx_message.tensor = self.config.speed
        self.tx.publish()

        state_msg = self.rx.message #  MessageReader 
        if state_msg is None:
            return "Nothing Received"
        if(DEBUG_MODE):
          print('\tMessage received with following Detials')
          # print('State Proto Pack Type {}'.format(type(state_msg.proto.pack)))
          print('State Proto Pack  {}'.format(state_msg.proto.pack) )

          # print('State Proto Element Type {}'.format(type(state_msg.proto.pack.elementType)) )
          # print('State Proto Element {}'.format(state_msg.proto.pack.elementType) ) # Float64
          
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
          print('\t__END__')

          # pt_message = self.rx.message
          # if pt_message is None:
          #     return "Nothing Received"
          # print("messages pt : ",pt_message)
        
        
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

'''
  Sight Slider.
  All Read Writable Table
'''