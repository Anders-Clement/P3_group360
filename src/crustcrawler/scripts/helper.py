#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

pos = [[],[],[],[]]
counter = 0
running = False


def callbackTraj(data):
    global running
    global firstRun

    if not running:
        return

    pos[0].append( rospy.get_rostime().to_sec() )
    pos[1].append( data.data[0] )
    pos[2].append( data.data[3] )
    pos[3].append( data.data[6] )

def listener():
    global running
    global firstRun
    global counter

    rospy.init_node('listener', anonymous=True)
    myo = rospy.Publisher('/myo_raw/myo_gest_str', String)
    rospy.Subscriber('/crustcrawler/trajectory', Float64MultiArray, callbackTraj)

    rate = rospy.Rate(10.0)
    d = rospy.Duration.from_sec(20)

    while not (rospy.is_shutdown()):
        if not running:
            myo.publish("THUMB_TO_PINKY")
            rate.sleep()
            myo.publish("REST")
            rate.sleep()
            
            counter -= -1
            if counter >= 3: 
                running = True
        else:
            rospy.loginfo("Recording started")
            rospy.sleep(d)

            rospy.loginfo("Recording stopped")

            fig, (ax1, ax2, ax3) = plt.subplots(3, sharey='col')

            ax1.plot(pos[0], pos[1], color='blue', label='setTheta')
            ax1.set_title("Joint 1")

            ax2.plot(pos[0], pos[2], color='blue', label='setTheta')
            ax2.set_title("Joint 2")

            ax3.plot(pos[0], pos[3], color='blue', label='setTheta')
            ax3.set_title("Joint 3")
            plt.show()
            return

if __name__ == '__main__':
    listener()