from __future__ import print_function

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from inputs import get_gamepad
import threading
import time

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.controller_msg =""
        self.i = 0

    def get_controller_msg(self, input_string):
        msg = String()
        self.controller_msg = input_string
        msg.data = self.controller_msg
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        #print(self.controller_msg)


def control_read(minimal_publisher_instance):
    x = 0
    analog_info=""
    feed =""
    while 1:
        
        events = get_gamepad()
        for event in events:
            analog_info = str(event.code)
            analog_info= analog_info.replace("ABS_","").replace("SYN_REPORT0","")
            analogState = str(event.state)
            feed = analog_info + analogState
            feed= feed.replace("SYN_REPORT0","")
            if "Y" or "X" in analog_info:
                feed=feed.replace("HAT0","")
                minimal_publisher_instance.get_controller_msg(str(feed))

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    control_thread = threading.Thread(target=control_read, args=(minimal_publisher,))
    control_thread.start()
    #print(minimal_publisher.get_controller_msg())
    rclpy.spin(minimal_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
    





    