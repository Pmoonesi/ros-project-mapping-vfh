#!/usr/bin/python3

from turtle import distance
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

        self.goal_x = rospy.get_param("/navigator/goal_x")
        self.goal_y = rospy.get_param("/navigator/goal_y")
        
        self.heading_errors = []
        
        ## pid params
        self.k_i = 0
        self.k_p = 0.6
        self.k_d = 7
        self.v = 0.6

        ## vfh params
        self.a = 1
        self.b = 0.25
        self.threshold = 3
        self.s_max = 6
        self.l = 2
        self.sector_size = 5

        self.dt = 0.05
        rate = 1 / self.dt
        self.r = rospy.Rate(rate)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    def get_value(self, distance):
        if distance == float('inf'): return 0
        return self.a - self.b * distance

    def get_range_values(self, ranges):
        return [self.get_value(distance) for d in ranges]

    def get_polar_density(self, ranges, sector_size):
        sectors = []
        start = -int(sector_size / 2)
        end = start + sector_size
        sectors.append(sum(ranges[start:] + ranges[:end]))
        for _ in range(1, int(len(ranges) / sector_size)):
            start = end
            end = start + sector_size
            sectors.append(sum(ranges[start:end]))
        return sectors

    def get_smoothed_array(self, ranges, l):
        smooth = []
        for i in range(len(ranges)):
            weighted_sum = 0
            for j in range(-l, l + 1):
                weighted_sum += ((l + 1) - abs(j)) * ranges[i + j]
            smooth.append(weighted_sum / (l + 1)**2)
        return smooth

    def get_raw_target_heading(self, odom_data: Odometry, goal_x, goal_y):
        current_x, current_y, _ = odom_data.pose.pose.position
        return math.atan2((goal_y - current_y), (goal_x - current_x))

    ## convert radian to sector
    def get_target_sector(self, target_heading, sector_size):
        target_degrees = target_heading * 180 / math.pi if target_heading >= 0 \
            else (target_heading + 2 * math.pi) * 180 / math.pi
        
        if target_degrees < sector_size / 2 or target_degrees > 360 - sector_size / 2:
            return 0
        
        return int(target_degrees / sector_size - 0.5)

    ## convert sector to radian
    def get_target_heading(self, sector):
        pass

    def get_goal_heading_vfh(self, laser_data: LaserScan, odom_data: Odometry):
        ranges = laser_data.ranges
        m_ranges = self.get_range_values(ranges)
        density = self.get_polar_density(m_ranges, self.sector_size)
        smooth = self.get_smoothed_array(density, self.l)

        target_heading = self.get_raw_target_heading(odom_data, self.goal_x, self.goal_y)
        target_sector = self.get_target_sector(target_heading, self.sector_size)

        if smooth[target_sector] < self.threshold:
            return target_heading

        if all(density >= self.threshold for density in smooth) == True:
            return target_heading

        ## select best heading
        le = len(smooth)
        l, r = (target_sector + 1) % le, (target_sector - 1) % le

        ## right hand side (index decreases)
        while smooth[r % le] >= self.threshold: r-= 1
        dn_r = df_r = r
        while smooth[df_r % le] < self.threshold: df_r -= 1

        ## left hand side (index increases)
        while smooth[l % le] >= self.threshold: l+= 1
        dn_l = df_l = l
        while smooth[df_l % le] < self.threshold: df_l += 1

        if abs(target_sector - dn_r) < abs(target_sector - dn_l):
            ## closest low right side is closer to goal
            dn = dn_r
            df = max(dn - self.s_max, df_r)
        else:
            ## closest low left side is closer to goal
            dn = dn_l
            df = min(dn + self.s_max, df_l)

        final_sector = (dn + df) / 2
        return self.get_target_heading(final_sector)


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