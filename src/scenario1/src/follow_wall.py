#!/usr/bin/python3

import math
from turtle import position
import rospy
import numpy as np
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

class PIDController():


    def __init__(self):
        
        rospy.init_node('wall_follower', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)
        self.mode = rospy.get_param("/follow_wall/mode")
    
        self.k_i = 0
        self.k_p = 3
        self.k_d = 22

        self.v = 0.1
        self.D = 0.5

        self.dt = 0.005
        rate = 1/self.dt
        
        self.r = rospy.Rate(rate)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.errs = []

    def reach_a_wall(self):
        go = Twist()
        go.angular.z = 0
        go.linear.x = self.v
        stop = Twist()
        laser_data = rospy.wait_for_message("/scan", LaserScan)
        d = self.distance_from_wall(laser_data)
        while (d > self.D + 0.15):
            self.cmd_vel.publish(go)
            laser_data = rospy.wait_for_message("/scan", LaserScan)
            d = self.distance_from_wall(laser_data)
        self.cmd_vel.publish(stop)          


    def distance_to_goal(self, odom_data):
        position = odom_data.pose.pose.position
        return math.sqrt((self.goalx - position.x)**2 + (self.goaly - position.y)**2)

    def distance_from_wall(self, laser_data):
        rng = laser_data.ranges[:180]
        d = min(rng)
        return d

    def distance_from_wall_from_sides(self, laser_data):
        front_rng = laser_data.ranges[:35]
        side_rng = laser_data.ranges[35:180]
        return min(front_rng), min(side_rng)

    def goal_heading(self, msg: Odometry):
        position = msg.pose.pose.position
        return float("{:.2f}".format(np.arctan2((self.goaly - position.y), (self.goalx - position.x))))
    
    def current_heading(self, msg: Odometry):
        orientation = msg.pose.pose.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        return float("{:.2f}".format(yaw))

    def angle_difference(self, current_heading, goal_heading):
        if current_heading > 0:
            sign = -1 if (current_heading - math.pi < goal_heading < current_heading) else +1
        else:
            sign = +1 if (current_heading + math.pi > goal_heading > current_heading) else -1
        return sign * (math.pi - abs(abs(current_heading - goal_heading) - math.pi))

    def window_interval(self, diff, r):
        angle = int(diff * 180 / math.pi)
        left_w = angle - r 
        if left_w < -180:
            left_w += 360
        right_w = angle + r
        if right_w > 180:
            right_w -= 360
        return left_w, right_w

    def is_window_clear(self, left_i, right_i, laser_data, odom_data):
        rng = laser_data.ranges
        distance = self.distance_to_goal(odom_data)

        if left_i * right_i > 0:
            return True if min(rng[left_i:right_i]) > distance else False
        else:
            left_interval = rng[left_i:]
            right_interval = rng[:right_i]
            return True if min(left_interval + right_interval) > distance else False

    def follow_wall(self):
        
        sum_i_theta = 0
        prev_theta_error = 0
        
        move_cmd = Twist()
        move_cmd.angular.z = 0
        move_cmd.linear.x = self.v

        while not rospy.is_shutdown():
            self.cmd_vel.publish(move_cmd)

            laser_data = rospy.wait_for_message("/scan", LaserScan)
            odom_data = rospy.wait_for_message("/odom", Odometry)
            
            if self.mode == "a":
                d = self.distance_from_wall(laser_data)    
                err = d - self.D
            else:
                f, s = self.distance_from_wall_from_sides(laser_data)
                err = min([f, s]) - self.D

            if err == float('inf'):
                err = 10
            else:    
                self.errs.append(err)
                
            sum_i_theta += err * self.dt
            
            P = self.k_p * err
            I = self.k_i * sum_i_theta
            D = self.k_d * (err - prev_theta_error)

            move_cmd.angular.z = P + I + D 
            prev_theta_error = err
            
            if self.mode == "a":
                move_cmd.linear.x = self.v          
            else:
                if f < self.D:
                    move_cmd.linear.x = 0
                else:
                    move_cmd.linear.x = self.v
                        
            self.r.sleep()

    def on_shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        plt.plot(list(range(len(self.errs))),
                    self.errs, label='errs')
        plt.axhline(y=0,color='R')
        plt.draw()
        plt.legend(loc="upper left", frameon=False)
        plt.savefig(f"errs_{self.k_p}_{self.k_d}_{self.k_i}.png")
        plt.show()

        rospy.sleep(1)
            

if __name__ == '__main__':
    try:
        pidc = PIDController()
        pidc.reach_a_wall()
        pidc.follow_wall()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")