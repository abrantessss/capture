#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from capture_msgs.srv import Claw as ClawSRV
from capture_msgs.msg import Claw as ClawMSG
from std_msgs.msg import Float64
    
class Claw(Node):
    def __init__(self, id, namespace):
        super().__init__('control_claw_py')

        self.service_name = '/' + str(namespace) + str(id) + '/capture/claw'
        self.publisher_name = '/' + str(namespace) + str(id) + '/capture/claw_angle'

        self.print_intro()

        #Create the service clients for the drone's claw
        self.srv = self.create_service(ClawSRV, self.service_name, self.handle_request)
        self.get_logger().info(f"Created Service: {self.service_name}")

        #Create publisher for gazebo plugin
        self.publisher = self.create_publisher(ClawMSG, self.publisher_name, 10)
        self.get_logger().info(f"Created Publisher: {self.publisher_name}")
        self.claw_arm_state = False #False: Retracted | True: Extended
        self.claw_fingers_state = False #False: Release | True: Catch
        self.arm_angle = 0.0
        self.fingers_angle = 0.0

    def handle_request(self, request, response):
        command = request.command #1: Claw Arm | 2: Claw Fingers
        msg = ClawMSG()
        if(command == 1): #Arm
            if(self.claw_arm_state): 
                self.claw_arm_state = False
                self.arm_angle = 0.0
                msg.arm = self.arm_angle
                msg.fingers = self.fingers_angle
                self.get_logger().info("Claw: Retracting")
            else:
                self.claw_arm_state = True
                self.arm_angle = -1.571
                msg.arm = self.arm_angle
                msg.fingers = self.fingers_angle
                self.get_logger().info("Claw: Extending")
        else:
            if(self.claw_fingers_state): 
                self.claw_fingers_state = False
                self.fingers_angle = 0.0
                msg.arm = self.arm_angle
                msg.fingers = self.fingers_angle
                self.get_logger().info("Claw: Releasing")
            else:
                self.claw_fingers_state = True
                self.fingers_angle = 1.4
                msg.arm = self.arm_angle
                msg.fingers = self.fingers_angle
                self.get_logger().info("Claw: Catching")

        self.publisher.publish(msg)
        self.get_logger().info(f"Claw Message Sent: Arm:{msg.arm} | Fingers:{msg.fingers}")

        response.success = True
        return response
    
    def print_intro(self):
        intro = """
                                                                                                               
           ,---.'|                                                   ,--.    ,----..                           
  ,----..  |   | :      ,---,                  .---.               ,--.'|   /   /   \      ,---,        ,---,. 
 /   /   \ :   : |     '  .' \                /. ./|           ,--,:  : |  /   .     :   .'  .' `\    ,'  .' | 
|   :     :|   ' :    /  ;    '.          .--'.  ' ;        ,`--.'`|  ' : .   /   ;.  \,---.'     \ ,---.'   | 
.   |  ;. /;   ; '   :  :       \        /__./ \ : |        |   :  :  | |.   ;   /  ` ;|   |  .`\  ||   |   .' 
.   ; /--` '   | |__ :  |   /\   \   .--'.  '   \| .        :   |   \ | :;   |  ; \ ; |:   : |  '  |:   :  |-, 
;   | ;    |   | :.'||  :  ' ;.   : /___/ \ |    ' '        |   : '  '; ||   :  | ; | '|   ' '  ;  ::   |  ;/| 
|   : |    '   :    ;|  |  ;/  \   \;   \  \;      :        '   ' ;.    ;.   |  ' ' ' :'   | ;  .  ||   :   .' 
.   | '___ |   |  ./ '  :  | \  \ ,' \   ;  `      |        |   | | \   |'   ;  \; /  ||   | :  |  '|   |  |-, 
'   ; : .'|;   : ;   |  |  '  '--'    .   \    .\  ;        '   : |  ; .' \   \  ',  / '   : | /  ; '   :  ;/| 
'   | '/  :|   ,/    |  :  :           \   \   ' \ |        |   | '`--'    ;   :    /  |   | '` ,/  |   |    \ 
|   :    / '---'     |  | ,'            :   '  |--"         '   : |         \   \ .'   ;   :  .'    |   :   .' 
 \   \ .'            `--''               \   \ ;            ;   |.'          `---`     |   ,.'      |   | ,'   
  `---`                                   '---"             '---'                      '---'        `----'     
                                                                                                               
        """
        self.get_logger().info(intro)
        self.get_logger().info("CLAW NODE initialized successfully.")

def main(args=None):
    rclpy.init(args=args)
    node = Claw(1, 'drone')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()