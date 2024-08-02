#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64,Float32
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
import tf2_py
import tf2_ros
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
odom_l_x=[]
odom_l_y=[]
def animate(i):
    find_ref_index()
    time1=rospy.get_time()
    FindOptimalControlActions()
    time2=rospy.get_time()
    ##switch between Coppelia and real-life interface
    #throttle,brake = PID(nom_U[0,0])
    throttle=nom_U[0,0]
    #print(nom_U)
    steer= nom_U[0,1]
    print("target velocity ="+str(nom_U[0,0])+", throttle ="+ str(throttle)+" steer = "+str(steer))
    #brakes_pub.publish(Float64(brake))
    throttle_pub.publish(Float32(throttle)) ##switch both to float 64 when using coppelia
    steer_pub.publish(Float32(steer))
    states=np.full((HOR+1,3),init_state)
    odom_l_x.append(init_state[0])
    odom_l_y.append(init_state[1])
    for i in range(HOR):
        states[i+1]=forwardModel(states[i],nom_U[i])
    '''
    obs_x=[]
    obs_y=[]
    for obs in obstacles:
        obs_x.append(obs.x)
        obs_y.append(obs.y)
    
    print("number of obstacles =" +str(len(obs_x)))
    '''
    time3=rospy.get_time()
    print("time for MPPI = "+str(time2-time1))
    print("time for everything else ="+str(time3-time2))
    ax1.clear()
    ax1.set_xlim((-30,30))
    ax1.set_ylim((-30,30))
    ax1.plot(x_path,y_path,color='black')
    ax1.plot(states[:,0],states[:,1],color ='blue')
    ax1.plot(odom_l_x,odom_l_y,color='orange')
    print("ref_index = "+str(ref_index))
    ax1.plot(x_path[ref_index],y_path[ref_index],color='white')

#MPPI paramters
DT=0.1
HOR=20
SAMPLES=400
LAMBDA=1
###
#Vehilce Parameters
L=1.1345 ##half the wheelbase
WT=0.8245 ##half the wheeltracks
MAXSTEER=30
MAXVEL=3
## global variables
init_state=np.array([0,0,np.pi/2])
nom_U=np.zeros((HOR,2))
x_path=[]
y_path=[]
ref_index=0
#obstacles=[] ## [obstacle_1, obstacle_2, obstacle_3,..etc]
vel=0
error_integral=0
error_old=0
got_path=False
#checkPoint=np.array([0,0]) ##[x,y]
#callbacks for coppelia
##PID for coppelia
KP = 0.018
KI = 0.0812
KD = 0.0
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
    #yaw += (math.pi / 2.0) ##for interfacing with Coppelia
    init_state[2]=yaw
##callbacks
'''
def checkPoints_callback(msg:Point):
    global got_path
    checkPoint[0]=msg.x
    checkPoint[1]=msg.y
    print("checkpoint = "+str(checkPoint))
    got_path=True
'''
'''
def obstacle_callback(msg:PointCloud):
    global obstacles
    obstacles.clear()
    for point in msg.points:
        obs=[]
        obs.append(point.x)
        obs.append(point.y)
        ##obs.append(point.z)
        obstacles.append(obstacles)
    print("inside")
'''
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
        if ref_index<len(x_path)-5:
            Range=range(ref_index,ref_index+5)
        else:
            Range=np.concatenate((range(ref_index,len(x_path)),range(0,5)))
        for i in Range:
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
    beta=-1*u[1]/2*(np.pi/180) ##multiply by negative 1 during real life demo
    x1=x0
    x1[0]=x1[0]+DT*u[0]*math.cos(x1[2]+beta)
    x1[1]=x1[1]+DT*u[0]*math.sin(x1[2]+beta)
    x1[2]=x1[2]+DT*(u[0]/L)*math.sin(beta)
    return x1
def calculateCost(Actions):
    global ref_index
    Actions=np.reshape(Actions,(HOR,2))
    states=np.full((HOR+1,3),init_state)
    for i in range(HOR):
        states[i+1]=forwardModel(states[i],Actions[i])
    cost = 0
    
    x_ref=[]
    y_ref=[]
    
    if int(HOR*DT*10/0.25)+1+ref_index<len(x_path):
        x_ref=x_path[ref_index:HOR*int(DT*MAXVEL/0.25)+ref_index+1:int(DT*MAXVEL/0.25)]
        y_ref=y_path[ref_index:HOR*int(DT*MAXVEL/0.25)+ref_index+1:int(DT*MAXVEL/0.25)]
    else:
        x_ref=x_path.take(range(ref_index,HOR*int(DT*MAXVEL/0.25)+ref_index+1,int(DT*MAXVEL/0.25)),mode='wrap')
        y_ref=y_path.take(range(ref_index,HOR*int(DT*MAXVEL/0.25)+ref_index+1,int(DT*MAXVEL/0.25)),mode='wrap')
    dist_x=abs(states[:,0]-x_ref)
    dist_y=abs(states[:,1]-y_ref)
    #Actions_added=np.insert(np.append(Actions,[0,0]),0,[0,0])
    #Rterm=abs(Actions_added[2:]-Actions_added[:-2]).sum()

    cost = dist_x.sum()+dist_y.sum()-0.5*np.hypot(states[HOR,0]-init_state[0],states[HOR,1]-init_state[1])
    ##Rterm may be removed it only slightly decreases oscillations
    return cost

def generateControlActions():
    perturbations=np.random.rand(HOR,2)-np.full((HOR,2),[0,0.5])
    scale=np.array([[MAXVEL,0],[0,MAXSTEER]])
    Du=perturbations@scale
    return Du
def FindOptimalControlActions():
    global nom_U
    costs = np.zeros(SAMPLES)
    Dus=np.zeros((SAMPLES,HOR,2))
    time1=rospy.get_time()
    for i in range(SAMPLES):
        Dus[i]=generateControlActions()
        costs[i]=calculateCost(nom_U+Dus[i])
    time2=rospy.get_time()
    print("the time taken for this for loop is"+str(time2-time1))
    #print("costs = "+str(costs))
    weights=np.exp(-(1/LAMBDA)*costs)
    #print("weights = "+str(weights))
    Du=np.einsum("i,ijk->jk",weights,Dus)*(1/(weights.sum(0)))
    print("Du ="+ str(Du))
    print("######")
    nom_U=nom_U+Du
    nom_U=np.clip(nom_U,[0,-1*MAXSTEER],[MAXVEL,MAXSTEER])
    print("cost of optimal control action is" +str(calculateCost(nom_U)))
    return nom_U
    '''Du_2=np.zeros((HOR,2))
    for i in range(SAMPLES):
        Du_2+=Dus[i]*weights[i]
    print("Final perturbation = "+str(Du))
    print("Final perturbation 2 = "+str(Du_2))'''
###
##Subscribers and publishers for Coppelia
#brakes_pub = rospy.Publisher('/brakes', Float64, queue_size=10)
state_sub=rospy.Subscriber('/odom',Odometry,callback=odom_callback,queue_size=10)
#throttle_pub=rospy.Publisher('/cmd_vel',Float64,queue_size=10) ##coppelia
#steer_pub=rospy.Publisher('/SteeringAngle',Float64,queue_size=10)  ##coppelia
#obstacle_sub=rospy.Subscriber('/obstacles', PointCloud, callback=obstacle_callback)
path_sub=rospy.Subscriber('/Path',Path,path_callback,queue_size=10)
throttle_pub = rospy.Publisher( '/in_Car_velocity_in_KM/H', Float32, queue_size=10)   ##realLife
steer_pub = rospy.Publisher('/in_Car_steering_in_degree', Float32, queue_size=10)     ##realLife
#check_point_sub=rospy.Subscriber('/checkpoint', Point, checkPoints_callback, queue_size=10)

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
        find_ref_index()
        FindOptimalControlActions()
        #throttle,brake = PID(nom_U[0,0])
        throttle=nom_U[0,0]
        #print(nom_U)
        steer= nom_U[0,1]
        print("target velocity ="+str(nom_U[0,0])+", throttle ="+ str(throttle)+", brakes ="+str(brake)+", steer = "+str(steer))
        brakes_pub.publish(Float64(brake))
        throttle_pub.publish(Float64(throttle))
        steer_pub.publish(Float64(steer))
        rate.sleep()
        '''