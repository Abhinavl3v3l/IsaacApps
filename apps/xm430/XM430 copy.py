from isaac import  *

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

JSON_FILE = r"apps/samples/xm430/xm430.app.json"

class XM430(Codelet):
    def start(self):
        self.rx = self.isaac_proto_rx("StateProto", "state_tx")
        self.rx = self.isaac_proto_rx("", "state_tx")
        DynamixelMotorsProto
        self.tick_on_message(self.rx)

    def tick(self):
        state_msg = self.rx.message #  MessageReader 
        if state_msg is None:
            return "Nothing Received"
        
        print('Message Type {}'.format(type(state_msg)) )
        print ('Message')

# class Pong(Codelet):
#     def start(self):

#         self.rx = self.isaac_proto_rx("PingProto", "pong_msg")
#         self.tick_on_message(self.rx)

#     def tick(self):
#         test_msg = self.rx.message #  MessageReader 
#         if test_msg is None:
#             return "Nothing Received"
        
#         print('Message Received {}'.format(test_msg.proto.message) )


def main(): 
    print('In Node XM430')
    app = Application(app_filename=JSON_FILE)
    print("JSON LOADED")
    app.nodes["xm430"].add(name="XM430",ctype=XM430)
    # app.nodes["pong"].add(name="Pong",ctype=Pong)
    app.run()

if __name__ == '__main__':
    main()
