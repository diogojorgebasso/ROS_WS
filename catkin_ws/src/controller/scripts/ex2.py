#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleController:
    def __init__(self):
        rospy.init_node('turtle_controller', anonymous=True)
       
        self.vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        self.radius = 0.75 
        self.obstacle_X = 3
        self.obstacle_Y = 3

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        self.linear_speed = 10.0   # Base speed for moving forward
        self.angular_speed = 4.0   # Base speed for turning
        self.distance_tolerance = 0.1  # Stop when close to target
        
        self.rate = rospy.Rate(10)  # 10 Hz

    def pose_callback(self, data):
        self.current_x = data.x
        self.current_y = data.y
        self.current_theta = data.theta

    def controller(self):
        # main controller
        positions = [0.75, 2.25, 3.75, 5.25, 6.75, 8.25, 9.75]

        for idx, j in enumerate(positions):
            # For even-index rows, traverse left-to-right; otherwise right-to-left.
            if idx % 2 == 0: 
                for i in positions:
                    self.move_to_target(i, j)
            else:
                for i in reversed(positions):
                    self.move_to_target(i, j)

    def get_distance_to_target(self):
        return math.sqrt((self.target_x - self.current_x) ** 2 + 
                         (self.target_y - self.current_y) ** 2)

    def get_angle_to_target(self):
        angle_to_target = math.atan2(self.target_y - self.current_y, 
                                     self.target_x - self.current_x)
        return angle_to_target - self.current_theta

    def normalize_angle(self, angle):
        # Normalize angle to be within [-pi, pi]
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def move_to_target(self, x, y):
        # Set the new target
        self.target_x = x
        self.target_y = y

        while not rospy.is_shutdown():
            distance = self.get_distance_to_target()
            angle = self.normalize_angle(self.get_angle_to_target())
            distance_obstacle = math.sqrt((self.obstacle_X - self.current_x) ** 2 +
                                          (self.obstacle_Y - self.current_y) ** 2)
                                                             
            cmd_vel = Twist()

            # Check if an obstacle is too close
            distance_tolerance_obstacle = 1.0
            if distance_obstacle < distance_tolerance_obstacle:
                rospy.loginfo(f"Obstacle detected! x: {self.obstacle_X}, y: {self.obstacle_Y}")
                rospy.loginfo("Contouring obstacle")
                
                # Calculate avoidance parameters
                avoidance_angular_speed = self.linear_speed / self.radius
                # Duration for a 180 degree (pi radians) turn along the semicircular path
                duration = math.pi / avoidance_angular_speed
                t0 = rospy.Time.now().to_sec()

                while (rospy.Time.now().to_sec() - t0) < duration and not rospy.is_shutdown():
                    cmd_vel.linear.x = self.linear_speed
                    cmd_vel.angular.z = avoidance_angular_speed
                    self.vel_pub.publish(cmd_vel)
                    self.rate.sleep()
                # After avoidance, continue to re-assess the target
                continue
            
            # If the target is reached, stop
            if distance < self.distance_tolerance:
                rospy.loginfo(f"Target reached! x: {self.target_x}, y: {self.target_y}")
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.vel_pub.publish(cmd_vel)
                break
            
            if abs(angle) > 0.2:
                cmd_vel.angular.z = self.angular_speed * (angle / abs(angle))
                cmd_vel.linear.x = 0.0  # Turn in place if the angle is too large
            else:
                cmd_vel.linear.x = min(self.linear_speed, distance)  # Move forward with speed limit
                cmd_vel.angular.z = 0.0
            
            self.vel_pub.publish(cmd_vel)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = TurtleController()
        controller.controller()
    except rospy.ROSInterruptException:
        pass
