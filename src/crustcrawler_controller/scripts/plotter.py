#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Int16MultiArray

pos = [[],[],[],[]]
actualPos = [[],[],[],[]]
running = False
firstRun = False

def callbackActual(data):
    global running
    global firstRun

    if not running:
        return

    actualPos[0].append(rospy.get_rostime().to_sec())
    actualPos[1].append( float(data.data[0]/1000.0) )
    actualPos[2].append( float(data.data[2]/1000.0) )
    actualPos[3].append( float(data.data[4]/1000.0) )

def callbackTraj(data):
    global running
    global firstRun

    if not firstRun:
        firstRun = True
        running = True

    if not running:
        return

    pos[0].append( rospy.get_rostime().to_sec() )
    pos[1].append( float(data.data[0]/1000.0) )
    pos[2].append( float(data.data[3]/1000.0) )
    pos[3].append( float(data.data[6]/1000.0) )

def listener():
    global running
    global firstRun

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/crustcrawler/getAngleVel', Int16MultiArray, callbackActual)
    rospy.Subscriber('/crustcrawler/trajectory', Int16MultiArray, callbackTraj)

    rate = rospy.Rate(10.0)
    d = rospy.Duration.from_sec(20)

    while not (rospy.is_shutdown()):
        if not running:
            rate.sleep()
        else:
            rospy.loginfo("Recording started")
            rospy.sleep(d)
            if (firstRun):
                running = False

            rospy.loginfo("Recording stopped")

            fig, (ax1, ax2, ax3) = plt.subplots(3, sharey='col')

            ax1.plot(actualPos[0], actualPos[1], color='red', label='Theta')
            ax1.plot(pos[0], pos[1], color='blue', label='setTheta')
            ax1.set_title("Joint 1")

            ax2.plot(actualPos[0], actualPos[2], color='red', label='Theta')
            ax2.plot(pos[0], pos[2], color='blue', label='setTheta')
            ax2.set_title("Joint 2")

            ax3.plot(actualPos[0], actualPos[3], color='red', label='Theta')
            ax3.plot(pos[0], pos[3], color='blue', label='setTheta')
            ax3.set_title("Joint 3")
            plt.show()
            return

if __name__ == '__main__':
    listener()