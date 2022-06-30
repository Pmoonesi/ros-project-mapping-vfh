#!/usr/bin/python3

import rospy
import tf
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class NavigationController():

    def __init__(self):
        
        rospy.init_node('wall_follower', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)
        
        self.heading_errors = []
        
        self.k_i = 0
        self.k_p = 0.6
        self.k_d = 7
        self.v = 0.6

        self.dt = 0.05
        rate = 1 / self.dt
        self.r = rospy.Rate(rate)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    def get_goal_heading_vfh(self, laser_data):
        pass

    def get_current_heading(self, msg: Odometry):
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

    def navigate(self):

        sum_i_theta = 0
        prev_theta_error = 0

        move_cmd = Twist()
        move_cmd.angular.z = 0
        move_cmd.linear.x = self.v
        
        while not rospy.is_shutdown():
            self.cmd_vel.publish(move_cmd)

            laser_data = rospy.wait_for_message("/scan", LaserScan)
            odom_data = rospy.wait_for_message("/odom", Odometry)

            goal_heading = self.get_goal_heading_vfh(laser_data)
            current_heading = self.get_current_heading(odom_data)
            ## get current and goal heading and calc err
            err = self.angle_difference(current_heading, goal_heading)

            self.heading_errors.append(err)
                
            sum_i_theta += err * self.dt
            
            P = self.k_p * err
            I = self.k_i * sum_i_theta
            D = self.k_d * (err - prev_theta_error)

            move_cmd.angular.z = P + I + D 
            move_cmd.linear.x = self.v          
            prev_theta_error = err
            
            self.r.sleep()



    def on_shutdown(self):
        rospy.loginfo("Stopping the robot...")
        rospy.sleep(1)

if __name__ == "__main__":
    try:
        controller = NavigationController()
        controller.navigate()
    except rospy.ROSInternalException:
        rospy.loginfo("Navigation terminated.")