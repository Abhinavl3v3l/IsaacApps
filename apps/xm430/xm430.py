from isaac import  *
import numpy as np

JSON_FILE = r"apps/samples/xm430/xm430.app.json"

class XM430_MG(Codelet):

    def start(self):
        self.tx = self.isaac_proto_tx("StateProto", "state")
        self.tick_periodically(0.2)

    '''
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
    '''


    def tick(self):
        tx_message = self.tx.init()
        # tx_message.proto.pack.elementType = np.float64
        tx_message.proto.pack.sizes = [1,1,1]
        tx_message.proto.pack.scanlineStride = 0
        tx_message.proto.pack.dataBufferIndex = 0
        tx_message.proto.schema = "StateProto"
        tx_message.proto.data = [1,1,1]
        self.tx.publish()

class XM430(Codelet):
    def start(self):
        self.rx = self.isaac_proto_rx("StateProto", "command")
        self.tick_on_message(self.rx)

    def tick(self):

        state_msg = self.rx.message #  MessageReader 
        if state_msg is None:
            return "Nothing Received"
        
        # print('\t_______START__________')
        # print('State Proto Pack Type {}'.format(type(state_msg.proto.pack)) )
        # print('State Proto Pack  {}'.format(state_msg.proto.pack) )

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


        # print('\t_______END__________')


def main(): 
    print('In Node XM430')
    app = Application(app_filename=JSON_FILE)
    print("JSON LOADED")
    app.nodes["xm430"].add(name="XM430",ctype=XM430)
    app.nodes["xm430_mg"].add(name="XM430_MG",ctype=XM430_MG)
    app.run()

if __name__ == '__main__':
    main()
