#!/usr/bin/env python
from __future__ import print_function
import rospy
from std_msgs.msg import String
from inputs import get_gamepad
import threading

class MinimalPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('minimal_publisher', anonymous=True)
        # Create a publisher
        self.publisher_ = rospy.Publisher('controller_topic', String, queue_size=10)
        self.controller_msg = ""

    def get_controller_msg(self, input_string):
        self.controller_msg = input_string
        msg = String()
        msg.data = self.controller_msg
        # Publish the message
        self.publisher_.publish(msg)
        rospy.loginfo('Publishing: "%s"' % msg.data)

def control_read(minimal_publisher_instance):
    feed = ""
    while not rospy.is_shutdown():
        events = get_gamepad()
        for event in events:
            analog_info = str(event.code).replace("ABS_", "").replace("SYN_REPORT0", "")
            analogState = str(event.state)
            feed = analog_info + analogState
            feed = feed.replace("SYN_REPORT0", "")
            if "Y" or "X" in analog_info:
                feed = feed.replace("HAT0", "")
                minimal_publisher_instance.get_controller_msg(feed)

def main():
    # Create the publisher instance
    minimal_publisher = MinimalPublisher()
    
    # Start the controller reading in a separate thread
    control_thread = threading.Thread(target=control_read, args=(minimal_publisher,))
    control_thread.daemon = True
    control_thread.start()
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
