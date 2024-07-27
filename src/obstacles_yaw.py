#!/usr/bin/env python3

import rospy
import tf
from sensor_msgs.msg import PointCloud2, PointField, PointCloud
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point32
import numpy as np
from sklearn.cluster import DBSCAN

class Lidar:
    def __init__(self):
        rospy.init_node("lidar_node", anonymous=True)
        self.cloud_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)
        self.cloud_pub = rospy.Publisher("/filtered_points", PointCloud2, queue_size=10)
        self.odom_sub = rospy.Subscriber("/aft_mapped_adjusted", Odometry, self.odom_callback)
        self.boundary_pub = rospy.Publisher("/obstacles", PointCloud, queue_size=10)
        self.points = [[], [], []]  # x, y, z
        self.min_distance_threshold = 0.5  # in meters
        self.max_distance_threshold = 5.0  # in meters
        self.vehicle_diagonal = 0.0
        self.current_odom = None

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.current_odom = (position, euler)

    def callback(self, ros_point_cloud):
        cloud_points = list(pc2.read_points(ros_point_cloud, skip_nans=True, field_names=("x", "y", "z")))

        rospy.loginfo("Received PointCloud with {} points.".format(len(cloud_points)))

        filtered_points = [[], [], []]
        for x, y, z in cloud_points:
            distance = np.sqrt(x**2 + y**2)
            if self.min_distance_threshold <= distance <= self.max_distance_threshold and z > -0.7 and z < 0.1:
                filtered_points[0].append(x)
                filtered_points[1].append(y)
                filtered_points[2].append(z)

        rospy.loginfo("Filtered {} points.".format(len(filtered_points[0])))
        self.points = filtered_points

        self.publish_filtered_points(filtered_points)
        self.cluster_points()

    def publish_filtered_points(self, filtered_points):
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"

        if filtered_points[0]:
            points = list(zip(filtered_points[0], filtered_points[1], filtered_points[2]))

            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1)
            ]
            point_cloud = pc2.create_cloud(header, fields, points)
            self.cloud_pub.publish(point_cloud)
            rospy.loginfo("Filtered points published.")

    def cluster_points(self):
        points_array = np.array(list(zip(self.points[0], self.points[1], self.points[2])))

        db = DBSCAN(eps=0.5, min_samples=10).fit(points_array)
        labels = db.labels_

        unique_labels = set(labels)
        centroids = []
        radii = []

        for k in unique_labels:
            if k != -1:
                class_member_mask = (labels == k)
                xy = points_array[class_member_mask]

                centroid = xy.mean(axis=0)
                centroids.append(centroid)

                distances = np.linalg.norm(xy - centroid, axis=1)
                max_distance = np.max(distances)
                radius = max_distance + self.vehicle_diagonal
                radii.append(radius)

        self.publish_boundaries(centroids, radii)

    def publish_boundaries(self, centroids, radii):
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"

        points = []
        position, euler = self.current_odom
        odom_x = position.x
        odom_y = position.y
        yaw = euler[2]

        for centroid, radius in zip(centroids, radii):
            point = Point32()
            point.x = centroid[0] * np.cos(yaw) - centroid[1] * np.sin(yaw) + odom_x
            point.y = centroid[0] * np.sin(yaw) + centroid[1] * np.cos(yaw) + odom_y
            point.z = radius
            points.append(point)

        boundary_msg = PointCloud()
        boundary_msg.header = header
        boundary_msg.points = points

        self.boundary_pub.publish(boundary_msg)
        rospy.loginfo("Boundary points published.")

if __name__ == "__main__":
    try:
        lidar = Lidar()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
