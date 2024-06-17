#!/usr/bin/env python

import rospy
from robot_control_class import RobotControl
import time
import math

class MazeNavigator:

    def __init__(self):
        self.robot = RobotControl()
        self.obstacle_threshold = 0.4  # Distance threshold for obstacle detection
        self.turn_speed = 1.0          # Turn speed
        self.move_speed = 0.2          # Move speed, lower for more stability
        self.turn_time = 1.5
        self.rate = rospy.Rate(10)     # Loop rate in Hz

    def is_obstacle_ahead(self):
        front_distance = self.robot.get_laser(360)
        return front_distance < self.obstacle_threshold

    def is_path_clear(self):
        laser_readings = self.robot.get_laser_full()
        left_clear = all(distance > self.obstacle_threshold for distance in laser_readings[480:719])
        right_clear = all(distance > self.obstacle_threshold for distance in laser_readings[0:240])
        return left_clear, right_clear

    def move_straight_with_correction(self):
        # Get the initial orientation
        initial_position, initial_yaw = self.robot.get_odom()

        self.robot.move_straight_time("forward", self.move_speed, 0.5)

        # After moving, get the current orientation
        current_position, current_yaw = self.robot.get_odom()

        # Calculate the yaw error
        yaw_error = current_yaw - initial_yaw

        # Correct the robot's orientation if there is a significant yaw error
        if abs(yaw_error) > 0.1:
            correction_angle = -math.degrees(yaw_error)
            self.robot.rotate(correction_angle)

    def navigate_maze(self):
        while not rospy.is_shutdown():
            if self.is_obstacle_ahead():
                self.robot.stop_robot()
                left_clear, right_clear = self.is_path_clear()
                
                if left_clear:
                    self.robot.rotate(90)
                elif right_clear:
                    self.robot.rotate(-90)
                else:
                    self.robot.rotate(180)
            else:
                self.move_straight_with_correction()
                time.sleep(0.1)
        
        self.robot.stop_robot()

if __name__ == '__main__':
    rospy.init_node('maze_navigator_node', anonymous=True)
    navigator = MazeNavigator()
    try:
        navigator.navigate_maze()
    except rospy.ROSInterruptException:
        pass
