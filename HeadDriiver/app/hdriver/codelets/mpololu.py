'''
@author:Abhinav Rana
'''
from isaac import *
import numpy as np
from genesis_app import create_genesis_app

# Json File Path
JSON_FILE = r"apps/hdriver/graphs/mpololu.app.json"

DEBUG_MODE = True


class POLOLU(Codelet):
    def start(self):
        print("In Start Function")

        self.value_rx = self.isaac_proto_rx("JsonProto", "command")
        self.tx = self.isaac_proto_tx("CompositeProto", "command")


        self.rx = self.isaac_proto_rx("CompositeProto", "state")
        self.tick_periodically(0.2)

        # self.config.target_ =       5000

        #Head
        #Dynamixel
        self.config.id3                 =   2000
        self.config.id4                 =   2000
        self.config.id5                 =   2000
        self.config.id6                 =   2000
        self.config.id7                 =   2000
        #pololu24
        self.config.eyebrow_ll0         =   6000
        self.config.eyebrow_lr1         =   6200
        self.config.lip_tc2             =   6400
        self.config.rightcheakneareyes3 =   6000
        self.config.eyebrow_c4          =   6200
        self.config.eyebrow_rl5         =   6000
        self.config.eyebrow_ll6         =   6000
        self.config.eye_pitch7          =   6000
        self.config.lip_tr8             =   6000
        self.config.eyebrow_rr9         =   6240
        self.config.lip_tl10            =   5864
        self.config.leftcheakneareye11  =   6000
        self.config.leftcheakneanose12  =   6000
        self.config.rightcheeknearnose13=   6000
        self.config.lip_ll14            =   6548
        self.config.lip_lc15            =   6000
        self.config.cheek_ll16          =   6000
        self.config.lip_lr17            =   5560
        self.config.cheek_lr_pitch18    =   6000
        self.config.cheek_ll_yaw19      =   6000
        self.config.cheek_lr_yaw20      =   6000
        self.config.toung_pitch21       =   6000
        self.config.na_22               =   0
        self.config.na_23               =   0
        #pololu
        self.config.na_0                =   0
        self.config.na_1                =   0
        self.config.eyelid_ll_pitch2    =   6000
        self.config.eyerightyaw3        =   6000
        self.config.eyelid_lu_pitch4    =   6000
        self.config.eyelib_ru_pitch5    =   6000
        self.config.eyelid_rl_pitch6    =   6000
        self.config.eye_l_yaw7          =   6000
        self.config.na_8               =   0
        self.config.na_9               =   0
        self.config.na_10              =   0
        self.config.na_11              =   0


    def tick(self):

        print("In Tick of TICK TICK")


         # Json proto to Composite Proto
        state_msg = self.value_rx.message  # MessageReader
        if state_msg is None:
            return "Nothing Received"
        if(DEBUG_MODE):
            print("Tensor  {}".format(state_msg.tensor))
        # vals   = state_msg.tensor.eval(session=tf.compat.v1.Session())
        vals = state_msg.tensor.numpy()
        print( "Type of Qunatity : ",type(vals))

        
        tx_message = self.tx.init()
        quantities = [
            ["d3", "position", 1], ["d4", "position", 1], ["d5", "position", 1], [
                "d6", "position", 1], ["d7", "position", 1],
            ["0_1", "position", 1], ["1_1", "position", 1], [
                "2_1", "position", 1],
            ["3_1", "position", 1], ["4_1", "position", 1], [
                "5_1", "position", 1],
            ["6_1", "position", 1], ["7_1", "position", 1], [
                "8_1", "position", 1],
            ["9_1", "position", 1], ["10_1", "position", 1], [
                "11_1", "position", 1],
            ["12_1", "position", 1], ["13_1", "position", 1], [
                "14_1", "position", 1],
            ["15_1", "position", 1], ["16_1", "position", 1], [
                "17_1", "position", 1],
            ["18_1", "position", 1], ["19_1", "position", 1], [
                "20_1", "position", 1],
            ["21_1", "position", 1], ["22_1", "position", 1], [
                "23_1", "position", 1],
            ["0_2", "position", 1], ["1_2", "position", 1], ["2_2", "position", 1],
            ["3_2", "position", 1], ["4_2", "position", 1], [
                "5_2", "position", 1],
            ["6_2", "position", 1], ["7_2", "position", 1], [
                "8_2", "position", 1],
            ["9_2", "position", 1], ["10_2", "position", 1], [
                "11_2", "position", 1],
        ]
        no_target = 0
        target  = self.config.target_
        # values = np.array([2000,2000,2000,2000,2000,
        # 5000,6000,5000,5000,5000,5000,
        # 5000,5000,5000,5000,5000,5000,
        # 5000,5000,5000,5000,5000,5000,
        # 5000,5000,5000,5000,5000,5000,
        # 5000,5000,5000,5000,5000,5000,
        # 5000,5000,5000,5000,5000,5000
        # ], dtype=np.dtype("float64"))

        values = np.array([self.config.id3, self.config.id4, self.config.id5,  self.config.id6, self.config.id7,
        self.config.eyebrow_ll0,
        self.config.eyebrow_lr1,
        self.config.lip_tc2,
        self.config.rightcheakneareyes3,
        self.config.eyebrow_c4,
        self.config.eyebrow_rl5,
        self.config.eyebrow_ll6,
        self.config.eye_pitch7,
        self.config.lip_tr8,
        self.config.eyebrow_rr9,
        self.config.lip_tl10,
        self.config.leftcheakneareye11,
        self.config.leftcheakneanose12,
        self.config.rightcheeknearnose13,
        self.config.lip_ll14,
        self.config.lip_lc15,
        self.config.cheek_ll16,
        self.config.lip_lr17,        
        self.config.cheek_lr_pitch18,
        self.config.cheek_ll_yaw19,  
        self.config.cheek_lr_yaw20, 
        self.config.toung_pitch21,   
        self.config.na_22,           
        self.config.na_23,
        self.config.na_0,
        self.config.na_1,
        self.config.eyelid_ll_pitch2,
        self.config.eyerightyaw3,    
        self.config.eyelid_lu_pitch4,
        self.config.eyelib_ru_pitch5,
        self.config.eyelid_rl_pitch6,
        self.config.eye_l_yaw7,      
        self.config.na_8,
        self.config.na_9,
        self.config.na_10,
        self.config.na_11], dtype=np.dtype("float64"))

       

        msg = Composite.create_composite_messageApplication(quantities, vals)
        # print('PRINITING MESSAGE CREATED ________',msg.proto.quantities)
        # time.sleep(3)
        tx_message.proto = msg.proto
        # print('MESSAGE BUFFER is ', msg.buffers)
        # print('MESSAGE HASH', msg.proto.schemaHash)
        tx_message.buffers = msg.buffers
        # # Publish
        self.tx.publish()



        state_msg = self.rx.message  # MessageReader
        if state_msg is None:
            return "Nothing Received"
        if(DEBUG_MODE):
            print("Tensor  {}".format(state_msg.tensor))



def main():
    print('Pololu')
    print('Loading JSON')
    app = create_genesis_app(app_filename=JSON_FILE)
    print("JSON LOADED")
    print('Adding Nodes')
    app.nodes["Mgen"].add(name="POLOLU", ctype=POLOLU)
    print('Nodes Added\nRunning Application')
    app.run()


if __name__ == '__main__':
    main()
