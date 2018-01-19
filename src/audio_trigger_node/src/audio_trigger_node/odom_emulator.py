#!/usr/bin/env python2.7

import rospy

from nav_msgs.msg import Odometry

def main(robotnum):
    rospy.init_node('odom_emulator' + str(robotnum))
    pubtopic = '/robot' + str(robotnum) + '/odometry/filtered'
    print 'Publishing to ' + pubtopic
    pub = rospy.Publisher(pubtopic, Odometry, queue_size = 1)

    locs = []
    if robotnum == 1:
        locs.append((2,18))
        locs.append((60,20))
    elif robotnum == 2:
        locs.append((101, 201))
        locs.append((151, 205))
    
    current_loc = 0
    while current_loc < len(locs):
        raw_input("Press enter to move Robot " + str(robotnum) + " to next trigger zone")
        msg = Odometry()
        msg.pose.pose.position.x = locs[current_loc][0]
        msg.pose.pose.position.y = locs[current_loc][1]

        pub.publish(msg)
        print("Published message")

        current_loc += 1
        