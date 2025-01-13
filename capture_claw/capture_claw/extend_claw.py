#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from capture_msgs.srv import ExtendClaw

class Claw(Node):
    def __init__(self, id, namespace):
        super().__init__('extend_claw_py')

        self.service_name = '/' + str(namespace) + str(id) + '/capture/extend_claw'

        self.print_intro()

        #Create the service clients for the drone's claw
        self.srv = self.create_service(ExtendClaw, self.service_name, self.handle_request)
        self.get_logger().info(f"Created Service: {self.service_name}")

    def handle_request(self, request, response):
        self.get_logger().info("Entrei")

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