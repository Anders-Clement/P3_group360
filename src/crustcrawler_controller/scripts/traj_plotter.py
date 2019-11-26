#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Int16MultiArray

traj_start = []
traj_end = []
pos = [[], [], [], [], [], []]
actualPos = [[], [], [], [], [], []]
vel = [[], [], [], [], [], []]
actualVel = [[], [], [], [], [], []]
acc = [[], [], [], [], [], []]
actualAcc = [[], [], [], [], [], []]
running = False
firstRun = False
start_time = 0.0


def callbackActual(data):
    global start_time

    if (start_time < 0.01):
        return

    actualPos[0].append(rospy.get_rostime().to_sec() - start_time)
    actualVel[0].append(rospy.get_rostime().to_sec() - start_time)

    for i in range(0,5):
        actualPos[i+1].append(float(data.data[0 + (i*2)]/1000.0))
        actualVel[i+1].append(float(data.data[1 + (i*2)]/1000.0))


def listener():
    global running
    global firstRun
    global traj_start
    global start_time

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/crustcrawler/getAngleVel',Int16MultiArray, callbackActual)
    traj_pub = rospy.Publisher('/crustcrawler/trajectory', Int16MultiArray,queue_size=10)
    rate = rospy.Rate(10.0)
    rospy.sleep(rospy.Duration.from_sec(2))

    traj_start = [0, 0, 0, 0, 0]
    traj_end = [1, 1, 1, 1, 1]

    tf = 20.0
    a = [[0 for y in range(5)] for x in range(5)]
    start_time = rospy.get_rostime().to_sec()
    indexForArray = 0


    for i in range(0, 5):  # calculates the 'a' coefficients using goal ang and vel
        a[0][i] = traj_start[i]
        a[1][i] = 0.0
        a[2][i] = 3.0 / (pow(tf, 2.0)) * (traj_end[i] - traj_start[i]) - 2.0 / tf * 0.0 - 1.0 / tf * 0.0
        a[3][i] = -2.0 / (pow(tf, 3.0)) * (traj_end[i] - traj_start[i]) + 1.0 / (pow(tf, 2.0)) * (0.0 + 0.0)

    while not rospy.is_shutdown():
        t = rospy.get_rostime().to_sec() - start_time
        if t > tf:
            break
        pos[0].append(t)
        vel[0].append(t)
        acc[0].append(t)

        print(t)
        
        for i in range(0, 5): # calculates the theta, thetadot and thetadotdot for all joints
            pos[i+1].append(a[0][i] + a[1][i] * t + a[2][i] * pow(t, 2.0) + a[3][i] * pow(t, 3.0))
            vel[i+1].append(a[1][i] + 2.0 * a[2][i] * t + 3.0 * a[3][i] * pow(t, 2.0))
            acc[i+1].append(2.0 * a[2][i] + 6.0 * a[3][i] * t)
        
        pub_array = Int16MultiArray()
        pub_array.data = [0]*15

        for i in range(1,6):
            pub_array.data[(i-1)*3] = (int(pos[i][indexForArray]*1000.0))
            pub_array.data[(i-1)*3+1] = (int(vel[i][indexForArray]*1000.0))
            pub_array.data[(i-1)*3+2] = (int(acc[i][indexForArray]*1000.0))

        traj_pub.publish(pub_array)

        indexForArray = indexForArray + 1
        rate.sleep()

    fig, ax = plt.subplots(5, sharey='col')
    
    for i in range(1,6):
        ax[i-1].plot(actualPos[0], actualPos[i], color='red', label='Theta')
        ax[i-1].plot(pos[0], pos[i], color='blue', label='setTheta')
        ax[i-1].set_title("Joint " + str(i))

    
    plt.show()

    return


if __name__ == '__main__':
    listener()
