#!/usr/bin/env python
import rospy
from control.msg import FourWheeler
from std_msgs.msg import String

key_mapping={'i':[1, 1], 'k':[-1, -1], 'l':[1, -1], 'j':[-1, 1], 'm':[0, 0]}

def on_receiving(msg, FourWheeler_publisher):
    if not key_mapping.has_key(msg.data[0]):
        return
    velocities=key_mapping[msg.data[0]]
    command=FourWheeler()
    command.left=velocities[0]
    command.right=velocities[1]
    FourWheeler_publisher.publish(command)
    print(command.left, command.right)

if __name__ == '__main__':
    rospy.init_node('keys_to_FourWheeler')
    FourWheeler_publisher = rospy.Publisher('FourWheeler', FourWheeler, queue_size=1)
    rospy.Subscriber('keys', String, on_receiving, FourWheeler_publisher)
    rospy.spin()
