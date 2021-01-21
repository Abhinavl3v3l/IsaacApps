from isaac import *

import cv2  as cv
import numpy as np 

JSON_FILE = r"/home/mrobotics2/Desktop/issac/sdk/apps/samples/ed/ed.app.json"

class ED(Codelet):
    '''
    PyCodelet for Edge Detection
    '''
    def start(self):
        print("EDGEDETECTOR START()...")
        self.rx = self.isaac_proto_rx("ImageProto", "iimage") #  RxHook
        self.tx = self.isaac_proto_tx("ImageProto", "oimage") #  TxHook
        self.tick_on_message(self.rx)
        self.cnt = 0


    def tick(self):
        print("EDGEDETECTOR TICK()...")
        img_msg1 = self.rx.message #  MessageReader 
        if img_msg1 is None:
            return 'Message From Camera Not Received...'

        print("Message Received from Camera...")
        expected_bytes_length = img_msg1.proto.rows * img_msg1.proto.rows * img_msg1.proto.channels
        print(f"BYTES LENGTH={expected_bytes_length}")

        #### OPENCV LOGIC ####
        print(f"TENSOR SHAPE={img_msg1.tensor.size}")
        opencv_image = cv.cvtColor(img_msg1.tensor, cv.COLOR_RGB2BGR) 
        edges = cv.Canny(opencv_image, 100, 200)


        # transmitting part
        message = self.tx.init() # MessageBuilder
        message.buffers = [np.array(edges)] 
        message.proto.dataBufferIndex = 0

        message.proto.rows = 480
        message.proto.cols = 640
        message.proto.channels = 1  

        message.proto.elementType = 'uint8'
        self.tx.publish() # this publishes message to respective channel
        print("MessagePublished from EdgeDetector to ImageViewer...")
 
def main():
    print("JSON FILE PROCESSING[...]")
    app = Application(app_filename=JSON_FILE) # instance of application
    print("JSON FILE PROCESSING COMPLETE[...]")
    app.nodes["e_d"].add(name="ED", ctype=ED) 
    app.run()


if __name__ == '__main__':
    main()
