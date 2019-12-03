#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Int16MultiArray

traj_start = []
traj_end = []
time = []
actualTime = []
pos = [[], [], [], [], []]
actualPos = [[], [], [], [], []]
vel = [[], [], [], [], []]
actualVel = [[], [], [], [], []]
acc = [[], [], [], [], []]
actualAcc = [[], [], [], [], []]
running = False
firstRun = False
start_time = 0.0


def callbackActual(data):
    global start_time

    if not running:
        return

    if start_time < 0.01 or rospy.get_rostime().to_sec() - start_time == 0:
        return

    actualTime.append(rospy.get_rostime().to_sec() - start_time)
    print("%.3f" % (rospy.get_rostime().to_sec() - start_time))

    for i in range(0,5):
        actualPos[i].append(float(data.data[0 + (i*2)]/1000.0))
        actualVel[i].append(float(data.data[1 + (i*2)]/1000.0))



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

    running = True

    traj_start = [0, 0, 0, 0, 0]
    traj_end = [1, 1, 1, 1, -1]      


    tf = 10.0
    a = [[0 for y in range(5)] for x in range(5)]
    start_time = rospy.get_rostime().to_sec()
    indexForArray = 0


    for i in range(0, 5):  # calculates the 'a' coefficients using goal-ang and -vel
        a[0][i] = traj_start[i]
        a[1][i] = 0.0
        a[2][i] = 3.0 / (pow(tf, 2.0)) * (traj_end[i] - traj_start[i]) - 2.0 / tf * 0.0 - 1.0 / tf * 0.0
        a[3][i] = -2.0 / (pow(tf, 3.0)) * (traj_end[i] - traj_start[i]) + 1.0 / (pow(tf, 2.0)) * (0.0 + 0.0)

    while not rospy.is_shutdown():
        t = rospy.get_rostime().to_sec() - start_time
        if t > tf:
            break
        time.append(t)


        
        for i in range(0, 5): # calculates the theta, thetadot and thetadotdot for all joints
            pos[i].append(a[0][i] + a[1][i] * t + a[2][i] * pow(t, 2.0) + a[3][i] * pow(t, 3.0))
            vel[i].append(-(a[1][i] + 2.0 * a[2][i] * t + 3.0 * a[3][i] * pow(t, 2.0)))
            acc[i].append(2.0 * a[2][i] + 6.0 * a[3][i] * t)
        
        pub_array = Int16MultiArray()
        pub_array.data = [0]*15

        for i in range(0,5):
            pub_array.data[(i)*3] = (int(pos[i][indexForArray]*1000.0))
            pub_array.data[(i)*3+1] = (int(vel[i][indexForArray]*1000.0))
            pub_array.data[(i)*3+2] = (int(acc[i][indexForArray]*1000.0))

        traj_pub.publish(pub_array)

        indexForArray = indexForArray + 1
        rate.sleep()

    running = False

    for j in range(0,actualVel[0].__len__()):
        for i in range(0,5):    
            if j == 0:
                actualAcc[i].append(actualVel[i][j]/actualTime[j])
            else:
                actualAcc[i].append((actualVel[i][j] - actualVel[i][j-1]) / (actualTime[j] - actualTime[j-1]))


    fig, ax = plt.subplots(5, 3, sharey='col',sharex='all')
    
    for i in range(0,5):
        ax[i][0].plot(actualTime, actualPos[i], color='red', label='Theta')
        ax[i][0].plot(time, pos[i], color='blue', label='setTheta')
        ax[i][0].set_ylabel("Joint " + str(i))
        ax[i][0].grid(color='k', alpha=0.3, linestyle='-', linewidth=0.5)

    for i in range(0,5):
        ax[i][1].plot(actualTime, actualVel[i], color='red', label='ThetaDot')
        ax[i][1].plot(time, vel[i], color='blue', label='setThetaDot')
        ax[i][1].grid(color='k', alpha=0.3, linestyle='-', linewidth=0.5)

    for i in range(0,5):
        ax[i][2].plot(actualTime, actualAcc[i], color='red', label='ThetaDotDot')
        ax[i][2].plot(time, acc[i], color='blue', label='setThetaDotDot')
        ax[i][2].grid(color='k', alpha=0.3, linestyle='-', linewidth=0.5)
        ax[i][2].set_ylim((-1.5, 1.5))

    ax[0][0].set_title("position")
    ax[0][1].set_title("velocity")
    ax[0][2].set_title("acceleration")

    plt.show()
    
    #for i in range(0,15):
    #    pub_array.data[i] = 0
    #traj_pub.publish(pub_array)

    return


if __name__ == '__main__':
    listener()
