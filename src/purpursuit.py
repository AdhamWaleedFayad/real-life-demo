#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Float64, Float32
from nav_msgs.msg import Odometry, Path
import math
import pandas as pd
from tf.transformations import euler_from_quaternion
from circle_fit import standardLSQ as curve_fit

data = [[], [], [], []]
time = 0.0
# Pure pursuit Constants #
WB = 1.2      #Wheel base
KDD = 0.1     #Velocity constant of lookahead distance
KS = 0.3        #Softening constant

# PID Constants #
KP = 0.08
KI = 0.012
KD = 0.0

# Simulation constants #
MU = 0.7                #Friction coefficient
G = 9.81                #Gravity
DT = 0.05               #Time step
MAX_VELOCITY = 5.0      #Maximum velocity               3.0
MAX_DISTANCE = 7.0      #Maximum distance to goal       9.0

def calc_distance(state_pos_x, state_pos_y, waypoint):

    waypoint_x = waypoint.x
    waypoint_y = waypoint.y
    dist = math.sqrt((state_pos_x - waypoint_x)**2 + (state_pos_y - waypoint_y)**2)

    return dist

class Trajectory:
    def __init__(self):
        # State Position of the vehicle #
        self.current_x = 0.0
        self.current_y = 0.0
        self.rear_x = 0.0
        self.rear_y = 0.0

        # State Orientation of the vehicle #
        self.ori_w = 0.0
        self.ori_x = 0.0
        self.ori_y = 0.0
        self.ori_z = 0.0

        # State Velocity of the vehicle #
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.velocity = 0.0

        self.waypoints = []             #Waypoints list
        
        self.Ld = 0.0                   #Look ahead distance
        self.desired_velocity = 0.0     #Target velocity
        
        # PID controller terms used in calculations #
        self.integral_error = 0.0
        self.old_error = 0.0
        
        self.old_nearest_index = None

    def waypointsCallback(self, path:Path):
        rospy.loginfo("<<<<<<New Path has been recieved>>>>>>")

        waypoints = []
        waypoints = path.poses
        
        if waypoints:
            self.waypoints = path.poses
            self.old_nearest_index = None

    def search_waypoint(self):
        if self.old_nearest_index is None:
            nearest_distance = float('inf')
            waypoint_index = 0
        
            half_len = (len(self.waypoints) / 2)
            for i in range(int(half_len)):
                distance = calc_distance(self.current_x, self.current_y, self.waypoints[i].pose.position)
                if distance < nearest_distance:
                    nearest_distance = distance
                    waypoint_index = i
    
            self.old_nearest_index = waypoint_index
        else:
            waypoint_index = self.old_nearest_index
            nearest_distance = calc_distance(self.current_x, self.current_y, self.waypoints[waypoint_index].pose.position)
        
            while (waypoint_index + 1) < len(self.waypoints):
                next_distance = calc_distance(self.current_x, self.current_y, self.waypoints[waypoint_index + 1].pose.position)
                if next_distance > nearest_distance:
                    break
                waypoint_index += 1
                nearest_distance = next_distance

            self.old_nearest_index = waypoint_index

        while self.Ld > calc_distance(self.current_x, self.current_y, self.waypoints[waypoint_index].pose.position):
            if (waypoint_index + 2) >= len(self.waypoints):
                break
            waypoint_index += 1

        return waypoint_index
    
    ### Call back function for state message ###
    def stateCallback(self, state:Odometry):
        global data, time
        rospy.loginfo("stateCallback has been called")

        self.current_x = state.pose.pose.position.x
        self.current_y = state.pose.pose.position.y

        self.ori_w = state.pose.pose.orientation.w
        self.ori_x = state.pose.pose.orientation.x
        self.ori_y = state.pose.pose.orientation.y
        self.ori_z = state.pose.pose.orientation.z

        self.vel_x = state.twist.twist.linear.x
        self.vel_y = state.twist.twist.linear.y

        self.velocity = math.sqrt((self.vel_x)**2 + (self.vel_y)**2)

        # throttle_pub = rospy.Publisher(
        #     '/cmd_vel', Float64, queue_size=10
        # )
        throttle_pub = rospy.Publisher(
            '/in_Car_velocity_in_KM/H', Float32, queue_size=10
        )
        steer_pub = rospy.Publisher(
            '/in_Car_steering_in_degree', Float32, queue_size=10
        )
        # brakes_pub = rospy.Publisher(
        #     '/brakes', Float64, queue_size=10
        # )

        # Calculate Lookahead distance using velocity (adaptive pure pursuit) #
        self.Ld = KDD * self.velocity + KS
        # rospy.loginfo("Ld: %f", self.Ld)

        # Search waypoints list to get the index of target waypoint #
        if self.waypoints:

            target_index = self.search_waypoint()
            # rospy.loginfo("Throttle: %f", throttle)
            throttle = 5.0

            # Calculate steering angle using adaptive pure pursuit algorithm #
            steering_angle = self.purepursuit(target_index)
            
            # rospy.loginfo("Velocity: %f", self.velocity)
            # rospy.loginfo("Target position: (%f, %f)", self.waypoints[target_index].pose.position.x, self.waypoints[target_index].pose.position.y)
            # rospy.loginfo("Target_index : %i", target_index)
            # rospy.loginfo("Current position: (%f, %f)", self.current_x, self.current_y)

        else:
            throttle = 0.0
            steering_angle = 0.0

        steering_angle_msg = Float32(steering_angle)
        steer_pub.publish(steering_angle_msg)

        throttle_msg = Float32(throttle)
        throttle_pub.publish(throttle_msg)

    def PID(self, targetIndex):
        next_points = []
        throttle = 0.0
        brakes = 0.0

        if targetIndex is None:
            return throttle, brakes

        if targetIndex < (len(self.waypoints) - 5):
            for i in range(4):
                next_points.append((self.waypoints[targetIndex + i].pose.position.x, self.waypoints[targetIndex + i].pose.position.y))
            xc, yc, r, sigma = curve_fit(next_points)

            curve_velocity = math.sqrt(MU * G * r) * 0.62

        elif targetIndex > (len(self.waypoints) - 2):
            curve_velocity = -1.0
            # brakes = 0.6
            # return throttle, brakes
        
        else:
            curve_velocity = MAX_VELOCITY

        relative_X = abs(self.waypoints[targetIndex].pose.position.x - self.current_x)
        R_pp = ((self.Ld) ** 2) / ((2.0 * relative_X) + 0.0001)
        
        purepursuit_velocity = math.sqrt(MU * G * R_pp) * 0.67

        self.desired_velocity = min(min(curve_velocity, purepursuit_velocity), MAX_VELOCITY)

        goal_distance = calc_distance(self.current_x, self.current_y, self.waypoints[-1].pose.position)
        
        if goal_distance < MAX_DISTANCE:
            self.desired_velocity = MAX_VELOCITY * (goal_distance / (1.5 * MAX_DISTANCE))       # 2.5

        # rospy.loginfo("Target Velocity = %f", self.desired_velocity)
        
        error = self.desired_velocity - self.velocity
        self.integral_error += error * DT
        derivative_error = (error - self.old_error) / DT

        throttle = (KP * error) + (KI * (self.integral_error)) + (KD * (derivative_error))

        self.old_error = error

        if throttle < -0.1:
            brakes = 0.65 * abs(throttle)

        throttle = max(min(1.0, throttle), 0.0)

        brakes = max(min(1.0, brakes), 0.0)
        
        return throttle, brakes

    def purepursuit(self, targetIndex):
        roll, pitch, yaw = euler_from_quaternion([self.ori_x, self.ori_y, self.ori_z, self.ori_w])

        # yaw += (math.pi/2.0)
        # rospy.loginfo("Yaw: %f", yaw)

        self.rear_y = self.current_y - ((WB) * math.sin(yaw))
        self.rear_x = self.current_x - ((WB) * math.cos(yaw))

        steering_angle = 0.0
                
        target_point = self.waypoints[targetIndex].pose.position
        target_point_x = target_point.x
        target_point_y = target_point.y

        relative_x = target_point_x - self.rear_x
        relative_y = target_point_y - self.rear_y

        alpha = math.atan2(relative_y , relative_x) - yaw

        steering_angle = math.atan2((2.0 * WB * math.sin(alpha)) / self.Ld , 1.0) * (180.0 / math.pi)

        steering_angle = max(min(25.0 , steering_angle) , -25.0)

        return steering_angle


if __name__ == '__main__':
    try:
        controller = Trajectory()
        rospy.init_node('controller', anonymous=True)
        rospy.loginfo("Controller has been intialized")
        
        rospy.Subscriber(
            '/Path', Path, controller.waypointsCallback, queue_size=10
        )

        rospy.Subscriber(
            '/aft_mapped_adjusted', Odometry, controller.stateCallback, queue_size=10
        )
        ###
        # /in_Car_velocity_in_KM/H          std_msgs/Float32        velocity
        # /in_Car_steering_in_degree        std_msgs/Float32        steering_angle
        # /aft_mapped_adjusted              nav_msgs/Odometry
        # /image_raw                        sensor_msgs/Image
        #  ###
        rospy.spin()

        # df_vel = pd.DataFrame(data)
        # csv_file_path_vel_throttle_brakes = '/home/mostafa/EVER-ws/vel_throttle_brakes_data_custom_track.csv'
        # df_vel.to_csv(csv_file_path_vel_throttle_brakes, index=False, header=False)
        
    except rospy.ROSInterruptException:
        pass
