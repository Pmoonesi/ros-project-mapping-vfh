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
        self.k_p = 1
        self.k_d = 10
        self.v = 0.2
        self.w = 0.1

        ## vfh params
        self.a = 1
        self.b = 0.25
        self.threshold = 2
        self.s_max = 10
        self.l = 2
        self.sector_size = 5

        self.dt = 0.1
        rate = 1 / self.dt
        self.r = rospy.Rate(rate)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    def get_distance_to_goal(self, odom_data: Odometry, goal_x, goal_y):
        pos = odom_data.pose.pose.position
        x, y = pos.x, pos.y
        return math.sqrt((x - goal_x)**2 + (y - goal_y)**2)

    def get_value(self, distance):
        if distance == float('inf'): return 0
        return self.a - self.b * distance

    def get_range_values(self, ranges):
        return [self.get_value(d) for d in ranges]

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

    def get_smooth_array(self, ranges, l):
        smooth = []
        for i in range(len(ranges)):
            weighted_sum = 0
            for j in range(-l, l + 1):
                weighted_sum += ((l + 1) - abs(j)) * ranges[(i + j) % len(ranges)]
            smooth.append(weighted_sum / (l + 1)**2)
        return smooth

    def get_current_heading(self, msg: Odometry):
        orientation = msg.pose.pose.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        return float("{:.2f}".format(yaw))

    def get_target_heading(self, odom_data: Odometry, goal_x, goal_y):
        pos = odom_data.pose.pose.position
        current_x, current_y = pos.x, pos.y
        return math.atan2((goal_y - current_y), (goal_x - current_x))

    def angle_difference(self, current_heading, goal_heading):
        if current_heading > 0:
            sign = -1 if (current_heading - math.pi < goal_heading < current_heading) else +1
        else:
            sign = +1 if (current_heading + math.pi > goal_heading > current_heading) else -1
        return sign * (math.pi - abs(abs(current_heading - goal_heading) - math.pi))

    def get_relative_target_heading(self, odom_data: Odometry, goal_x, goal_y):
        ch = self.get_current_heading(odom_data)
        th = self.get_target_heading(odom_data, goal_x, goal_y)
        return self.angle_difference(ch, th)

    ## convert radian to sector
    def get_target_sector(self, target_heading, sector_size):
        target_degrees = target_heading * 180 / math.pi if target_heading >= 0 \
            else (target_heading + 2 * math.pi) * 180 / math.pi
        
        if target_degrees < sector_size / 2 or target_degrees > 360 - sector_size / 2:
            return 0
        
        return int(target_degrees / sector_size - 0.5)

    ## convert sector to radian
    def get_relative_final_heading(self, sector, sector_size):
        in_degrees = sector * sector_size
        return in_degrees * math.pi / 180 if (in_degrees >= 0 and in_degrees < 180) else in_degrees * math.pi / 180 - 2 * math.pi


    def get_goal_heading_vfh(self, laser_data: LaserScan, odom_data: Odometry):
        ranges = laser_data.ranges
        m_ranges = self.get_range_values(ranges)
        density = self.get_polar_density(m_ranges, self.sector_size)
        smooth = self.get_smooth_array(density, self.l)

        target_heading = self.get_relative_target_heading(odom_data, self.goal_x, self.goal_y)
        target_sector = self.get_target_sector(target_heading, self.sector_size)

        print(f'target sector: {target_sector} density: {smooth[target_sector]}')

        if smooth[target_sector] < self.threshold:
            return target_heading, smooth[target_sector]

        if all(density >= self.threshold for density in smooth) == True:
            print("threshold alert")
            return target_heading, self.threshold

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

        final_sector = ((dn + df) / 2 - 0.5) % le
        final_heading = self.get_relative_final_heading(final_sector, self.sector_size)
        print(f'relative target: {target_heading}\tfinal relative target: {final_heading}')
        return final_heading, smooth[int(final_sector)]
        

    def navigate(self):

        move_cmd = Twist()
        move_cmd.angular.z = 0

        stop = Twist()
        stop.angular.z = 0
        stop.linear.x = 0

        rotate_cmd = Twist()
        rotate_cmd.linear.x = 0
        
        while not rospy.is_shutdown():
            self.cmd_vel.publish(stop)

            laser_data = rospy.wait_for_message("/scan", LaserScan)
            odom_data = rospy.wait_for_message("/odom", Odometry)
            distance_to_goal = self.get_distance_to_goal(odom_data, self.goal_x, self.goal_y)

            if distance_to_goal < 0.2:
                print("reached the goal!")
                break

            ## rotate
            err, density = self.get_goal_heading_vfh(laser_data, odom_data)
            last_heading = self.get_current_heading(odom_data)

            while abs(err) > 0.2:
                rotate_cmd.angular.z = self.w if err > 0 else -self.w
                print(f"error is {err} so we're rotating with {rotate_cmd.angular.z}")
                self.cmd_vel.publish(rotate_cmd)
                rospy.sleep(1)
                new_odom_data = rospy.wait_for_message("/odom", Odometry)
                current_heading = self.get_current_heading(new_odom_data)
                err -= self.angle_difference(last_heading, current_heading)
                last_heading = current_heading

            self.cmd_vel.publish(stop)
            rospy.sleep(0.1)

            ## move forward
            move_cmd.linear.x = self.v
            self.cmd_vel.publish(move_cmd)
            togo = (self.a - density / self.sector_size) / self.b if \
                (self.a - density / self.sector_size) / self.b < distance_to_goal else distance_to_goal
            t = 0.5 * togo / self.v
            rospy.sleep(t) 
            
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