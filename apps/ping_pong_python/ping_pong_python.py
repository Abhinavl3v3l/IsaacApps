from isaac import *

JSON_FILE = r"/home/mrobotics2/Desktop/issac/sdk/apps/tutorials/ping_pong_python/ping_pong_python.app.json"
class Ping(Codelet):
    def start(self):
        self.tx = self.isaac_proto_tx("PingProto", "ping_msg")
        self.tick_periodically(0.1)

    def tick(self):
        # print('Printing : ',self.config.message)
        tx_message = self.tx.init()
        tx_message.proto.message = "Message_PINGING"
        self.tx.publish()

class Pong(Codelet):
    def start(self):

        self.rx = self.isaac_proto_rx("PingProto", "pong_msg")
        self.tick_on_message(self.rx)

    def tick(self):
        test_msg = self.rx.message #  MessageReader 
        if test_msg is None:
            return "Nothing Received"
        
        print('Message Received {}'.format(test_msg.proto.message) )


def main(): 
    app = Application(app_filename=JSON_FILE)
    print("JSON LOADED")
    app.nodes["ping"].add(name="Ping",ctype=Ping)
    app.nodes["pong"].add(name="Pong",ctype=Pong)
    app.run()

if __name__ == '__main__':
    main()
