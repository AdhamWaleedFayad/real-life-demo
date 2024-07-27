#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Bool, Float32
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import PoseStamped, PointStamped, Point
import math
import heapq
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import numpy as np

# Simulation constants #
WB = 2.269              #Wheel base
MU = 0.7                #Friction coefficient
G = 9.81                #Gravity
DT = 0.05               #Time step
KV = 0.2                #Velocity constant (step size)      0.3
STEP_SIZE = 0.5         #Step size                          1.1
RANGE_DIST = 1.0       #Range distance                     10.0

state_pos_x = 0.0
state_pos_y = 0.0

start_pos_x = 0.0
start_pos_y = 0.0
state_yaw = 0.0
velocity = 0.0

plotted_path_x = [[], [], []]
plotted_path_y = [[], [], []]

obstacles = []
obstacles_distance = 0.0

goal_node = None
goal_reached = False
# for plotting
style.use('fivethirtyeight')
fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)

def animate(i):
    global state_pos_x, state_pos_y, plotted_path_x, plotted_path_y
    for i in range(len(plotted_path_x)):
        ax1.plot(plotted_path_x[i], plotted_path_y[i], 'b-')
    # ax1.clear()

    # if plotted_path_x and plotted_path_y:
    #     for i in range(len(plotted_path_x)):
    #         ax1.plot(plotted_path_x[i], plotted_path_y[i], 'bo')
    #     plotted_path_x.clear()
    #     plotted_path_y.clear()

    ax1.plot(state_pos_x, state_pos_y, 'go')

    # for i, obstacle in enumerate(obstacles):
    #     ax1.plot(obstacle.x, obstacle.y, 'ro')
    # ax1.set_xlim(-20.0, 20.0)
    # ax1.set_ylim(0.0, 80.0)


class Node:
    def __init__(self, x, y, g_cost=0.0, h_cost=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.g_cost = g_cost                        # Cost from start to current Node
        self.h_cost = h_cost                        # Heuristic cost from current Node to goal
        self.f_cost = self.g_cost + self.h_cost     # Total cost of the Node
        self.theta = theta                          # Orientation of the Node

def get_distance(current:Node, neighbor:Node):

    return math.sqrt((neighbor.x - current.x)**2 + (neighbor.y - current.y)**2)

# def checkPoints_callback(data:PointStamped):
#     global goal_node, goal_reached
#     msg = data.point
    
#     if goal_reached:
#         goal_node = None
#     else:
#         goal_node = Node(msg.x, msg.y)

# def is_goal_callback(data:Bool):
#     global goal_reached
#     goal_reached = data.data

def velocity_callback(data:Float32):
    global velocity
    velocity = data.data


def odom_callback(msg:Odometry):
    global state_pos_x, state_pos_y, start_pos_x, start_pos_y, state_yaw, velocity, goal_node

    state_pos_x = (msg.pose.pose.position.x)
    state_pos_y = (msg.pose.pose.position.y)

    ori_x = msg.pose.pose.orientation.x
    ori_y = msg.pose.pose.orientation.y
    ori_z = msg.pose.pose.orientation.z
    ori_w = msg.pose.pose.orientation.w

    # vel_x = msg.twist.twist.linear.x
    # vel_y = msg.twist.twist.linear.y

    # velocity = math.sqrt(vel_x**2 + vel_y**2)
    velocity = 5.0 * (5.0/18.0)

    roll, pitch, yaw = euler_from_quaternion([ori_x, ori_y, ori_z, ori_w])
    state_yaw = yaw
    rospy.loginfo("Yaw orientation: %f", state_yaw)

    start_pos_x = state_pos_x #+ ((0.2) * math.cos(state_yaw))
    start_pos_y = state_pos_y #+ ((0.2) * math.sin(state_yaw))

    path_pub = rospy.Publisher(
        '/Path', Path, queue_size=10
    )
        
    path = Path()
    path.header.frame_id = "map"

    rospy.loginfo("Generating path...")
    goal_node = Node(10.0, 0.0, theta=0.0)
    # if goal_node is not None:
    best_path = hybrid_astar(goal_node)

    for i, node in enumerate(best_path):
        pose = PoseStamped()
        pose.pose.position.x = node[0]
        pose.pose.position.y = node[1]
        path.poses.append(pose)

    path_pub.publish(path)

    # else:
    #     empty_path = Path()
    #     empty_path.header.frame_id = "map"
    #     empty_path.poses = []
    #     path_pub.publish(empty_path)
    
    rospy.loginfo("Path has been published.")

def obstacle_callback(msg:PointCloud):
    global obstacles
    obstacles = msg.points

def heuristic_cost_estimate(neighbor:Node, goal:Node):
    global obstacles, obstacles_distance
    
    heuristic_cost = ((goal.x - neighbor.x)**2 + (goal.y - neighbor.y)**2) #+ (math.sqrt(obstacles_distance) / 100.0)
    return heuristic_cost

def is_not_valid_node(node:Node):
    global obstacles, obstacles_distance

    # for i, obstacle in enumerate(obstacles):
    #     obstacle_node = Node(obstacle.x, obstacle.y)
    #     obstacle_distance = get_distance(node, obstacle_node)
    #     obstacles_distance += obstacle_distance

    #     if obstacle_distance < (obstacle.z):
    #         return True
    return False

def collision(neighbor:Node):
    return True

def generate_neighbors(current:Node):
    global velocity
    neighbors = []
    
    l_delta = -20.0*(math.pi/180.0)
    r_delta =  20.0*(math.pi/180.0)

    steering_angles = [l_delta, 0.0, r_delta] # Possible steering angles

    # Generate Hybrid A* neighbors
    for delta in steering_angles:
        theta = current.theta + delta
        x = current.x + (((velocity * KV) + STEP_SIZE) * np.cos(theta))
        y = current.y + (((velocity * KV) + STEP_SIZE) * np.sin(theta))
        rospy.loginfo("x: %f, y: %f, theta: %f", x, y, theta)

        neighbor = Node(x, y, theta = theta)
        neighbors.append(neighbor)

    return neighbors

def reconstruct_path(parents:dict, current_node:Node):
    global plotted_path_x, plotted_path_y

    optimal_path = []
    while current_node is not None:
        optimal_path.append((current_node.x, current_node.y))
        
        plotted_path_x.append(current_node.x)
        plotted_path_y.append(current_node.y)

        current_node = parents[(current_node.x, current_node.y)]

    optimal_path = optimal_path[::-1]

    return optimal_path

def hybrid_astar(goal:Node):
    global state_pos_x, state_pos_y, start_pos_x, start_pos_y, state_yaw, plotted_path_x, plotted_path_y, obstacles_distance

    open_list = []
    closed_list = []
    came_from = dict()
    g_score = dict()
    h_score = dict()

    start = Node(start_pos_x, start_pos_y, theta=state_yaw)

    came_from = {(start.x, start.y): None}
    g_score = {(start.x, start.y): 0.0}
    h_score = {(start.x, start.y): heuristic_cost_estimate(start, goal)}

    start.h_cost = h_score[(start.x, start.y)]
    start.f_cost = start.g_cost + start.h_cost

    heapq.heappush(open_list, (start.f_cost, start.g_cost, start_pos_x, start_pos_y, state_yaw))
    rospy.loginfo("Starting Hybrid A* algorithm...")

    closed_list = [start]
    
    while open_list:
        current_tuple = heapq.heappop(open_list)
        # rospy.loginfo("Current Node: ({}, {})".format(current_tuple[2], current_tuple[3]))
        # rospy.loginfo("Current Node's f_cost: {}".format(current_tuple[0]))
        # rospy.loginfo("Current Node's g_cost: {}".format(current_tuple[1]))
        
        current = Node(
            current_tuple[2], current_tuple[3], g_score[(current_tuple[2], current_tuple[3])], h_score[(current_tuple[2], current_tuple[3])], current_tuple[4]
        )

        if (get_distance(current, goal) < 0.5):
            came_from[(goal.x, goal.y)] = current
            optimal_path = reconstruct_path(came_from, current)
            return optimal_path
        
        if get_distance(start, current) > RANGE_DIST:
            optimal_path = reconstruct_path(came_from, current)
            return optimal_path
        
        if current not in closed_list:
            closed_list.append(current)

        neighbors = generate_neighbors(current)

        for i, neighbor in enumerate(neighbors):

            if is_not_valid_node(neighbor):
                continue

            neighbor.g_cost = current.g_cost + get_distance(current, neighbor)
            neighbor.h_cost = heuristic_cost_estimate(neighbor, goal)
            neighbor.f_cost = neighbor.g_cost + neighbor.h_cost

            heapq.heappush(open_list, (neighbor.f_cost, neighbor.g_cost, neighbor.x, neighbor.y, neighbor.theta))

            came_from[(neighbor.x, neighbor.y)] = current
            g_score[(neighbor.x, neighbor.y)] = neighbor.g_cost
            h_score[(neighbor.x, neighbor.y)] = neighbor.h_cost
            
            obstacles_distance = 0.0
            plotted_path_x[neighbors.index(neighbor)] = [neighbor.x, current.x]
            plotted_path_y[neighbors.index(neighbor)] = [neighbor.y, current.y]
    current_tuple = min(h_score, key = h_score.get)
    current = came_from[(current_tuple[0], current_tuple[1])]
    
    return reconstruct_path(came_from, current)

if __name__ == '__main__':
    try:
        rospy.init_node('hybrid_astar', anonymous=True)
        rospy.sleep(1)
        rospy.loginfo("Hybrid A* node has been initialized.")

        # rospy.Subscriber(
        #     '/checkpoint', PointStamped, checkPoints_callback, queue_size=10
        # )

        # rospy.Subscriber(
        #     '/goal', Bool, is_goal_callback, queue_size=10
        # )
        
        rospy.Subscriber(
            '/obstacles', PointCloud, obstacle_callback, queue_size=10
        )

        rospy.Subscriber(
            '/aft_mapped_adjusted', Odometry, odom_callback, queue_size=10
        )
        ani = animation.FuncAnimation(fig, animate, interval=1000*DT)
        plt.show()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
