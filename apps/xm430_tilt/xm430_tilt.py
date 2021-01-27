from isaac import  *
import numpy as np

'''
# Commands commands for a set of dynamixel motors running for example as a dasy chain.

struct DynamixelMotorsProto {
# Motor protos
  struct MotorProto {
    # Motor ID
    id @0: Int64;
    # Current position
    position @1: Int16;
    # Tick in milliseconds since startup
    tick @2: Int64;
    # Is the servo moving
    moving @3: Bool;
  }
  # A single command per motor
  motors @0: List(MotorProto);
}
'''

JSON_FILE = r"apps/samples/xm430_tilt/xm430_tilt.app.json"

# class StateSender(Codelet):
#     def start(self):
#         self.tx =self.isaac_proto_tx("StateProto","state")
#         self.tick_periodically(1.1)

#     def tick(self):

#         '''
        
#         # A n-dimensional tensor
#         struct TensorProto {
#         # Type of channel elements
#         elementType @0: ElementType;
#         # Dimensions of the tensor
#         sizes @1: List(Int32);
#         # deprecated - not used anymore
#         scanlineStride @2: UInt32;
#         # Index of buffer which contains the tensor data
#         dataBufferIndex @3: UInt16;
#         }

#         struct StateProto {
#         # A densely packed representation of the state data as floating point numbers. The lowest rank
#         # indicates state element dimension, the second rank is time for a potential timeseries, and
#         # the third rank is for the batch index.
#         pack @0: TensorProto;
#         # The schema describing the format of the state vector
#         schema @1: Text;
#         # Alternative way to pass the data (for python)
#         data @2: List(Float64);
#         }

#         # ( elementType = float64,
#         #     sizes = [1, 1, 1],
#         #     scanlineStride = 0,
#         #     dataBufferIndex = 0 )
#         '''
#         # tx_message = self.tx.init()
#         # tx_message.proto.pack.elementType = np.float64
#         # tx_message.proto.pack.sizes = [1,1,1]
#         # self.tx.publish()

    

class XM430_TILT(Codelet):
    def start(self):
        self.rx = self.isaac_proto_rx("StateProto", "state_rx")
        self.tick_on_message(self.rx)

    def tick(self):
        state_msg = self.rx.message #  MessageReader 
        if state_msg is None:
            return "Nothing Received"
        
        print('\t_______START__________')
        print('State Proto  {}'.format(type(state_msg.proto.pack)) )

        print('State Proto Element Type {}'.format(type(state_msg.proto.pack.elementType)) )
        print('State Proto Element {}'.format(state_msg.proto.pack.elementType) ) # Float64
        
        print('State Proto Size Type {}'.format(type(state_msg.proto.pack.sizes)) )
        print('State Proto Size {}'.format(state_msg.proto.pack.sizes) )
        
        print(' SP Scan Line Stride Type {}'.format(type(state_msg.proto.pack.scanlineStride)))
        print(' SP Scan Line Stride {}'.format(state_msg.proto.pack.scanlineStride))
        
        print(' State Proto Buffer Index Type {}'.format(type(state_msg.proto.pack.dataBufferIndex)))
        print(' State Proto Buffer Index {}'.format(state_msg.proto.pack.dataBufferIndex))

        print('Schema {}'.format(state_msg.proto.schema))


        print('Data Type {}'.format(type(state_msg.proto.data)))
        print('Data {}'.format(state_msg.proto.data))


        print('\t_______END__________')



def main(): 
    print('In Node XM430_TILT')
    app = Application(app_filename=JSON_FILE)
    print("JSON LOADED")
    app.nodes["xm430_tilt"].add(name="XM430_TILT",ctype=XM430_TILT)
    app.run()

if __name__ == '__main__':
    main()
