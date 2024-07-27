#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
import numpy as np
import math
checkpoint_idx=0
goal_reached = False
def odom_callback(pos):
    global checkpoint_idx, goal_reached
    x=pos.pose.pose.position.x
    y=pos.pose.pose.position.y
    if started == True and checkpoint_idx<len(check_points_x)-1:
        if ((x-check_points_x[checkpoint_idx])**2+(y-check_points_y[checkpoint_idx])**2)<MIN_DIST**2:
            checkpoint_idx=checkpoint_idx+1
    checkpoint = PointStamped()
    checkpoint.header.frame_id = "map"
    checkpoint.point.x=check_points_x[checkpoint_idx]
    checkpoint.point.y=check_points_y[checkpoint_idx]
    check_points_pub.publish(checkpoint)

    if checkpoint_idx == len(check_points_x) - 1 and ((x-check_points_x[checkpoint_idx])**2+(y-check_points_y[checkpoint_idx])**2)<GOAL_DIST**2:
        goal_reached = True
    goal_pub.publish(goal_reached)

odom_sub=rospy.Subscriber('/filtered_odom',Odometry,odom_callback,queue_size=10)
check_points_pub=rospy.Publisher('/checkpoint',PointStamped,queue_size=10)
goal_pub = rospy.Publisher('/goal', Bool, queue_size=10)
TRACK=2
DS=40.0             #20.0 for straight track(1), 13.0 for circular track(2)
MIN_DIST=8.0        #7.0 for straight track(1), 9.0 for circular track(2)
GOAL_DIST=2.0
# check_points = [[1.0, 20], [1.0, 40], [1.0, 60], [1.0, 80], [1.0, 100], [1.0, 120], [1.0, 140], [1.0, 160], [1.0, 180]]
#check_points=[[10, 21],[26, 27], [41, 20], [52, 1], [45, -17], [23, -28], [7, -20], [0, 0]]
# check_points = [[51.0, 0.0], [0.0, 0.0]]
# check_points=[[3.35, 12.5], [12.5, 21.65], [25, 25], [37.5, 21.65], [46.65, 12.5], [51, 0], [46.65, -12.5], [37.5, -21.65],[25, -25], [12.5, -21.65], [3.35, -12.5], [0, 0]]
check_points_x=[]
check_points_y=[]
if TRACK==1:
    check_points_y=np.linspace(0,80,int((80/DS)+1))
    check_points_x=np.zeros(len(check_points_y)) 
if TRACK==2:
    check_points_x = [0.0, -4.0, -8.5, -9.0, -13.0 -27.0, -60.0]
    check_points_y = [50.0, 61.0, 55.0, -20.0, -25.0, -21.5, -21.5]
    # check_points_y_1=np.linspace(0,90,int((80/DS)+1))
    # check_points_x_1=np.full(len(check_points_y_1), 0.0)
    # check_points_y_2=np.linspace(120,160,int((120/DS)+1))
    # check_points_x_2=np.full(len(check_points_y_2), 6.0)
    # check_points_x=np.concatenate((check_points_x_1,check_points_x_2))
    # check_points_y=np.concatenate((check_points_y_1,check_points_y_2))
if TRACK==3:
    check_points_x = [12.6, -12.8]
    check_points_y = [2.8, 0.0]
    # angles=np.linspace(np.pi,-1*np.pi,int(2*np.pi*28.0/DS)+1)
    # for angle in angles :
    #     check_points_x.append(25.0+28.0*np.cos(angle))
    #     check_points_y.append(28.0*np.sin(angle))
started=False

if __name__ == '__main__':
    try:
        rospy.init_node('Global_Planning', anonymous=True)
        # for i in range(2):
        checkpoint = PointStamped()
        checkpoint.header.frame_id = "map"
        checkpoint.point.x=check_points_x[0]
        checkpoint.point.y=check_points_y[0]
        check_points_pub.publish(checkpoint)
        #plt.plot(check_points_x,check_points_y)
        #plt.show()
        started= True
        rospy.spin()
    except rospy.ROSInterruptException:
        pass