#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
import math
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point,PoseStamped
DS=1.5  ## distance between each point
S=7.5    ##distance between centre of circle and corner
R=7.125 ##radius of mid-circle
max_angle=math.acos(R/S)
corner_1_x=7.125*np.cos(max_angle)-S
corner_1_y=7.125*np.sin(max_angle)
corners_x=np.array([corner_1_x,-1*corner_1_x,-1*corner_1_x,corner_1_x])
corners_y=np.array([corner_1_y,-1*corner_1_y,corner_1_y,-1*corner_1_y])
x_circle_1=[]
y_circle_1=[]
t_line=np.linspace(0,1,int(math.hypot(2*corner_1_x,2*corner_1_y)/DS))
line_x_1=[]
line_y_1=[]
line_x_2=[]
line_y_2=[]
if __name__ == '__main__' :
    rospy.init_node("Path_gen")
    path_pub=rospy.Publisher('/Path',Path,queue_size=10)
    angles=np.linspace(np.pi,max_angle,int((2*np.pi*R*((np.pi-max_angle)/(2*np.pi)))/(DS)))
    for angle in angles:
        x_circle_1.append(7.125*np.cos(angle)-S)
        y_circle_1.append(7.125*np.sin(angle))
    for t in t_line:
        line_x_1.append((1-t)*corners_x[0]+(t)*corners_x[1])
        line_y_1.append((1-t)*corners_y[0]+(t)*corners_y[1])
    for t in t_line:
        line_x_2.append((1-t)*corners_x[2]+(t)*corners_x[3])
        line_y_2.append((1-t)*corners_y[2]+(t)*corners_y[3])
    angles=np.linspace(max_angle-np.pi,np.pi-max_angle,int((2*np.pi*R*(2*(np.pi-max_angle)/(2*np.pi)))/(DS)))
    x_circle_2=[]
    y_circle_2=[]
    for angle in angles:
        x_circle_2.append(7.125*np.cos(angle)+S)
        y_circle_2.append(7.125*np.sin(angle))
    angles=np.linspace(-1*max_angle,-1*np.pi,int((2*np.pi*R*((np.pi-max_angle)/(2*np.pi)))/(DS)))
    x_circle_3=[]
    y_circle_3=[]
    for angle in angles:
        x_circle_3.append(7.125*np.cos(angle)-S)
        y_circle_3.append(7.125*np.sin(angle))
    x_path=np.concatenate((x_circle_1,line_x_1,x_circle_2,line_x_2,x_circle_3))
    y_path=np.concatenate((y_circle_1,line_y_1,y_circle_2,line_y_2,y_circle_3))
    path=Path()
    for i in range(len(x_path)):
        pose = PoseStamped()
        pose.pose.position.x=x_path[i]
        pose.pose.position.y=y_path[i]
        path.poses.append(pose)
    print(path)
    while not rospy.is_shutdown():
        path_pub.publish(path)
        rospy.sleep(0.5)
    '''plt.plot(x_path,y_path)
    plt.show()
    '''
    # rospy.spin()