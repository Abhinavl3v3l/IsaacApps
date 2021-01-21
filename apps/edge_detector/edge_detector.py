
from isaac import *
import numpy as np
import cv2 as cv

JSON_FILE = r"/home/mrobotics2/Desktop/issac/sdk/apps/samples/edge_detector/edge_detector.app.json"

class EdgeDetector(Codelet):
    # Two function start and ping are overloaded.
    
    def start(self):

        # This part will be run once in the beginning of the program

        '''
        #An image produced for example by a camera
        struct ImageProto {
            # Type of channel elements
            elementType @0: ElementType;
            # Number of rows in the image
            rows @1: UInt16;
            # Number of columns in the image
            cols @2: UInt16;
            # Number of channels per pixel
            channels @3: UInt16;
            # Index of buffer which contains the image data
            dataBufferIndex @4: UInt16;
        }
        '''

        # So every node will either recieve(Rx), transmit(Tx) or both.
        print('Inside Ticker')
        self.rx = self.isaac_proto_rx("ImageProto", "iimage")
        self.tx = self.isaac_proto_tx("ImageProto", "oimage")
        self.tick_on_message(self.rx)

    def tick(self):
        # This will run every tick  
        rx_message = self.rx.message
        if rx_message is None:
            return "Nothing Received"

        # Message(Image) 

        print('-------Type of Rx Message --------- : ',type(rx_message))        
        rows = rx_message.proto.rows
        cols = rx_message.proto.cols
        channels = rx_message.proto.channels
        print('Rows {}, Columns {} and  Channels {} of image '.format(rows,cols,channels))

        # Compute Edge
        #### OPENCV LOGIC ####
        print(f"TENSOR SHAPE={rx_message.tensor.size}")
        opencv_image = cv.cvtColor(rx_message.tensor, cv.COLOR_RGB2BGR) 
        edges = cv.Canny(opencv_image, 100, 200)

        # Publish control command
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
    print('INSIDE MAIN')
    app = Application(app_filename=JSON_FILE)
    print("JSON FILE PROCESSING COMPLETE[...]")
    app.nodes["edge_detector"].add(name="EdgeDetector",ctype=EdgeDetector)
    app.run()


if __name__ == '__main__':
    main()

        

        