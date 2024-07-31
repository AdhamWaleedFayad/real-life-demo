#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud
from tf.transformations import euler_from_quaternion
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches
from matplotlib import style
#state is [x,y,heading]
#control Action is [velocity, steer]


#Adham
##for plotting
style.use('fivethirtyeight')
fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)
##
def animate(i):
    find_ref_index()
    FindOptimalControlActions()
    throttle,brake = PID(nom_U[0,0])
    #print(nom_U)
    steer= nom_U[0,1]
    print("target velocity ="+str(nom_U[0,0])+", throttle ="+ str(throttle)+", brakes ="+str(brake)+", steer = "+str(steer))
    brakes_pub.publish(Float64(brake))
    throttle_pub.publish(Float64(throttle))
    steer_pub.publish(Float64(steer))
    states=np.full((HOR+1,3),init_state)
    for i in range(HOR):
        states[i+1]=forwardModel(states[i],nom_U[i])
    ax1.clear()
    ax1.set_xlim((-15,15))
    ax1.set_ylim((-15,15))
    ax1.plot(states[:,0],states[:,1])


#MPPI paramters
DT=0.1
HOR=20
SAMPLES=500
LAMBDA=10
###
#Vehilce Parameters
L=1
MAXSTEER=30
MAXVEL=3
## global variables
init_state=np.array([0,0,np.pi/2])
nom_U=np.zeros((HOR,2))
obstacles=[]
vel=0
error_integral=0
error_old=0
got_path=False
#callbacks for coppelia
##PID for coppelia
KP = 0.018
KI = 0.0812
KD = 0.0
x_path=[]
y_path=[]
ref_index=0
def PID(target_vel:float):
    global vel
    global error_integral
    global error_old
    error = target_vel-vel
    error_integral=error_integral+error*DT
    error_differential=(error-error_old)/DT
    throttle=KP*error+KI*error_integral+KD*error_differential
    error_old=error
    if throttle < -0.1:
        brake = 0.7 * abs(throttle)
    else:
        brake=0.0
    throttle=max(0.0,min(throttle,1))
    brake=max(0.0,min(brake,1))
    return throttle,brake
def odom_callback(state:Odometry):
    global init_state,vel
    init_state[0]=state.pose.pose.position.x
    init_state[1]=state.pose.pose.position.y
    ori_w = state.pose.pose.orientation.w
    ori_x = state.pose.pose.orientation.x
    ori_y = state.pose.pose.orientation.y
    ori_z = state.pose.pose.orientation.z
    vel= math.hypot(state.twist.twist.linear.x,state.twist.twist.linear.y)
    roll, pitch, yaw = euler_from_quaternion([ori_x, ori_y, ori_z, ori_w])
    print("current velocity = "+str(vel))
    yaw += (math.pi / 2.0) ##for interfacing with Coppelia
    init_state[2]=yaw
def obstacle_callback(msg:PointCloud):
    global obstacles
    obstacles=msg.points
def path_callback(msg:Path):
    global x_path
    global y_path
    global got_path
    x_path=[]
    y_path=[]
    for pose in msg.poses:
        x_path.append(pose.pose.position.x)
        y_path.append(pose.pose.position.y)
    x_path=np.array(x_path)
    y_path=np.array(y_path)
    got_path=True
    print("got_path")
def find_ref_index():
    global ref_index
    if got_path ==True:
        min_dist=100.0
        for i in range(ref_index,ref_index+5):
            if i>=len(x_path):
                break
            dx=float(x_path[i])-init_state[0]
            dy=float(y_path[i])-init_state[1]
            dist=math.hypot(dx,dy)
            if dist<=min_dist:
                min_dist=dist
                ref_index=i
        return i
    else :
        ref_index=0
        return 0
##
##MPPI Functions
def forwardModel(x0, u):
    u=np.clip(u,[0,-1*MAXSTEER],[MAXVEL,MAXSTEER])
    beta=u[1]/2*(np.pi/180)
    x1=x0
    x1[0]=x1[0]+DT*u[0]*math.cos(x1[2]+beta)
    x1[1]=x1[1]+DT*u[0]*math.sin(x1[2]+beta)
    x1[2]=x1[2]+DT*(u[0]/L)*math.sin(beta)
    return x1
def calculateCost(Actions):
    global ref_index
    states=np.full((HOR+1,3),init_state)
    for i in range(HOR):
        states[i+1]=forwardModel(states[i],Actions[i])
    cost = 0
    dist_x=abs(states[:,0]-x_path[ref_index:int(HOR*DT*10/0.25)+1+ref_index:int(DT*10/0.25)])
    dist_y=abs(states[:,1]-y_path[ref_index:int(HOR*DT*10/0.25)+1+ref_index:int(DT*10/0.25)])
    '''
    for state in states:
        for obs in obstacles:
            dist=math.hypot(state[0]-obs[0],state[1]-obs[1])
            if dist<obs[2]:
                return np.inf
            else:
                sum_dist+=dist**2
    '''
    ##print("distance x summation =" +str(dist_x.sum()))
    ##print("distance x summation =" +str(dist_x.sum()))
    cost = dist_x.sum()+dist_y.sum()-1*np.hypot(states[HOR,0]-init_state[0],states[HOR,1]-init_state[1])
    ##to be edited later
    '''
        stateCost=(state[1]-state[0])+5
        cost+=stateCost
        ###
        '''
    return cost

def generateControlActions():
    perturbations=np.random.rand(HOR,2)-np.full((HOR,2),[0,0.5])
    scale=np.array([[MAXVEL,0],[0,2*MAXSTEER]])
    Du=perturbations@scale
    return Du
def FindOptimalControlActions():
    global nom_U
    costs = np.zeros(SAMPLES)
    Dus=np.zeros((SAMPLES,HOR,2))
    for i in range(SAMPLES):
        Dus[i]=generateControlActions()
        costs[i]=calculateCost(nom_U+Dus[i])
    #print("costs = "+str(costs))
    weights=np.exp(-(1/LAMBDA)*costs)
    #print("weights = "+str(weights))
    Du=np.einsum("i,ijk->jk",weights,Dus)*(1/(weights.sum(0)))
    print("Du ="+ str(Du))
    print("######")
    nom_U=nom_U+Du
    nom_U=np.clip(nom_U,[0,-1*MAXSTEER],[MAXVEL,MAXSTEER])
    return nom_U
    '''Du_2=np.zeros((HOR,2))
    for i in range(SAMPLES):
        Du_2+=Dus[i]*weights[i]
    print("Final perturbation = "+str(Du))
    print("Final perturbation 2 = "+str(Du_2))'''
###
##Subscribers and publishers for Coppelia
brakes_pub = rospy.Publisher('/brakes', Float64, queue_size=10)
state_sub=rospy.Subscriber('/odom',Odometry,callback=odom_callback,queue_size=10)
throttle_pub=rospy.Publisher('/cmd_vel',Float64,queue_size=10)
steer_pub=rospy.Publisher('/SteeringAngle',Float64,queue_size=10)
obstacle_sub=rospy.Subscriber("/boundary_points", PointCloud, callback=obstacle_callback)
path_sub=rospy.Subscriber('/Path',Path,path_callback,queue_size=10)
##

if __name__ == '__main__' :
    rospy.init_node("MPPI")
    rate=rospy.Rate(1/DT)
    while not rospy.is_shutdown():
        ##for animating
        if got_path==True:
            ##
            ani = animation.FuncAnimation(fig, animate, interval=1000*DT)
            plt.show()
        ##
        '''
        ##for debugging
        if got_path==True:
            Actions_1=np.full((HOR,2),[10,0])
            Actions_2=np.full((HOR,2),[10,30])
            Actions_3=np.full((HOR,2),[10,-30])
            print("The cost for action 1 is "+ str(calculateCost(Actions_1)))
            print("The cost for action 2 is "+ str(calculateCost(Actions_2)))
            print("The cost for action 3 is "+ str(calculateCost(Actions_3)))
            break
        '''
        ##for running without animation
        '''
        #find_ref_index()
        FindOptimalControlActions()
        throttle,brake = PID(nom_U[0,0])
        #print(nom_U)
        steer= nom_U[0,1]
        print("target velocity ="+str(nom_U[0,0])+", throttle ="+ str(throttle)+", brakes ="+str(brake)+", steer = "+str(steer))
        brakes_pub.publish(Float64(brake))
        throttle_pub.publish(Float64(throttle))
        steer_pub.publish(Float64(steer))
        rate.sleep()
        '''