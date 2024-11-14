# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int16

# Callback function for the steering data
def steering_callback(msg):
    print("Steering data received: {}".format(msg.data))

def main():
    rospy.init_node('steering_listener', anonymous=True)
    rospy.Subscriber("/ctrlcmd_steering", Int16, steering_callback)
    rospy.spin()

if __name__ == "__main__":
    main()