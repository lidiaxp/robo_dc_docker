#!/usr/bin/env python

import rospy

from math import atan2, sqrt, pi, cos, sin
import time

from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion

from simple_a_star import AStarPlanner

class ObstacleAvoider:
    def __init__(self, goal):
        rospy.init_node('obstacle_avoider', anonymous=True)


        rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.vel_pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=10)

        self.rate = rospy.Rate(10)  
        self.distance_tolerance = 0.1
        self.linear_speed = 0.5  
        self.angular_speed = 1.0 

        self.goal = goal
        self.current_position = Point(0, 0, 0)
        self.current_orientation = 0.0  
        self.obstacle_detected = False
        self.obstacle_distance = float('inf')
        self.safe_distance = 1.5

        self.grid_size = 2.0  
        self.robot_radius = 1.0     
        self.ox, self.oy, self.oxy = [], [], []
        self.curr_traj = []

        self.waypoints = [goal]
        self.current_waypoint = 0
        self.stop_flag = False

    
    def move_to_waypoint(self, target):
        twist = Twist()
        
        while not rospy.is_shutdown():
            distance = sqrt((target.x - self.current_position.x) ** 2 + (target.y - self.current_position.y) ** 2)
            angle_to_target = atan2(target.y - self.current_position.y, target.x - self.current_position.x)
            angle_diff = angle_to_target - self.current_orientation

            if self.stop_flag:
                rospy.loginfo("Stop trajectory.")
                break
            
            if distance < self.distance_tolerance:
                rospy.loginfo("Waypoint reached!")
                break
            
            twist.linear.x = self.linear_speed * (distance > self.distance_tolerance)
            twist.angular.z = self.angular_speed * angle_diff
            
            self.vel_pub.publish(twist)
            self.rate.sleep()


    def follow_waypoints(self):
        rospy.loginfo("Start - Following Waypoints...")

        for waypoint in self.waypoints:
            rospy.loginfo(f"Going to the waypoint: {waypoint}")
            self.move_to_waypoint(waypoint)

        rospy.loginfo("Trajectory completed!")
        

    def stop(self):
        self.vel_pub.publish(Twist())  
        self.stop_flag = True


    def scan_callback(self, msg):
        angle_increment = msg.angle_increment
        angle = msg.angle_min

        try:
            for r in msg.ranges:
                obs_x = int(r * cos(angle) + self.current_position.x)
                obs_y = int(r * sin(angle) + self.current_position.y)
                
                if [obs_x, obs_y] not in self.oxy:
                    self.ox.append(obs_x)
                    self.oy.append(obs_y)
                    self.oxy.append([obs_x, obs_y])
                
                angle += angle_increment
        except:
            pass

        print('----------------------')
        print(len(self.ox), len(self.oy), len(self.oxy))
        print(self.oxy)

        self.obstacle_distance = min(msg.ranges)
        self.obstacle_detected = self.obstacle_distance < self.safe_distance


    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position

        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.current_orientation = yaw


    def timer_callback(self, event):
        if self.obstacle_detected:
            if self.check_collision():
                rospy.loginfo("Obstacle detected! Taking evasive action.")
                self.stop()
                a_star = AStarPlanner(self.ox, self.oy, self.grid_size, self.robot_radius)
                rx, ry = a_star.planning(self.current_position.x, self.current_position.y, self.goal.x, self.goal.y)
                
                self.stop_flag = False
                
                self.curr_traj = []
                for wp_x, wp_y in zip(rx, ry):
                    rospy.loginfo(f"Going to the waypoint: {wp_x}, {wp_y}")
                    self.move_to_waypoint(Point(wp_x, wp_y, 0))
                    self.curr_traj.append([wp_x, wp_y])


    def move_to_goal(self):
        self.curr_traj = [[[self.current_position.x, self.current_position.y], [self.goal.x, self.goal.y]]]
        waypoints = [
            self.goal
        ]
        self.waypoints = waypoints
        self.follow_waypoints()

        while not rospy.is_shutdown():
            distance = sqrt((self.goal.x - self.current_position.x) ** 2 + (self.goal.y - self.current_position.y) ** 2)
            if distance < self.distance_tolerance:
                break
            

    def run(self):
        rospy.loginfo("Start - Moving to Goal with Obstacle Avoidance...")

        self.move_to_goal()

        rospy.loginfo("Goal reached!")
        self.vel_pub.publish(Twist())  


    def check_collision(self):
        collision_threshold = 0.5  

        for i in range(len(self.curr_traj) - 1):
            start = self.curr_traj[i]
            end = self.curr_traj[i + 1]

            for obs in self.oxy:
                obs_x, obs_y = obs

                dist = self.point_to_line_distance(obs_x, obs_y, start, end)

                if dist < collision_threshold:
                    rospy.logwarn("Collision detected with obstacle at ({}, {})".format(obs_x, obs_y))
                    return True  

        rospy.loginfo("No collision detected along the trajectory.")
        return False  


    def point_to_line_distance(self, px, py, start, end):
        x1, y1 = start
        x2, y2 = end

        line_len_sq = (x2 - x1) ** 2 + (y2 - y1) ** 2
        if line_len_sq == 0:
            return sqrt((px - x1) ** 2 + (py - y1) ** 2)

        t = max(0, min(1, ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / line_len_sq))

        closest_x = x1 + t * (x2 - x1)
        closest_y = y1 + t * (y2 - y1)

        return sqrt((px - closest_x) ** 2 + (py - closest_y) ** 2)


if __name__ == '__main__':
    try:
        goal = Point(float(input("Enter goal X: ")), float(input("Enter goal Y: ")), 0)

        avoider = ObstacleAvoider(goal)
        avoider.run()

    except rospy.ROSInterruptException:
        pass
