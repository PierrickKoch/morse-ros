#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

topic = None

def wander(message):
    assert(len(message.ranges) >= 30)
    mid = len(message.ranges) // 2
    cmd = Twist()
    # halt if an object is less than 2m in a 30deg angle
    halt = False
    for distance_to_object in message.ranges[mid-15:mid+15]:
        if distance_to_object < 2:
            halt = True
            break
    if halt:
        # we go to the highest-range side scanned
        if sum(message.ranges[:mid]) > sum(message.ranges[mid:]):
            cmd.angular.z = -1
        else:
            cmd.angular.z = +1
    else:
        cmd.linear.x = 1
    # publish twist
    topic.publish(cmd)

if __name__ == '__main__':
    # ./nodes/wander.py cmd:=/robot/motion laser:=/robot/sick
    rospy.init_node('wander')
    rospy.loginfo("wander rospy initialized")
    topic = rospy.Publisher('cmd', Twist)
    # see: ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html
    rospy.Subscriber('laser', LaserScan, wander)
    rospy.spin() # block while ROS-node is up, Ctrl+C to stop
