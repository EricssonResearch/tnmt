import rospy
import numpy as np
import math
import os

from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from respawnGoal import Respawn
from std_srvs.srv import Empty

import pdb;
import subprocess

from std_msgs.msg import Empty
import time


class Environment():
    def __init__(self, action_size):
        self.goal_x = 1.03
        self.goal_y = -2.04
        self.direction_of_movement = 0
        self.scan_values = None
        # True in simulation false in robot.
        self.initGoal = False
        self.max_scan_value = float(3.5)
        self.action_size = action_size
        self.get_goalbox = False
        self.current_position = Pose()
        self.pub_cmd_vel = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        self.respawn_goal = Respawn()

    def getGoalDistace(self):
        goal_distance = round(math.hypot(self.goal_x - self.current_position.x, self.goal_y - self.current_position.y), 2)
        return goal_distance

    def getOdometry(self, odom):
            self.current_position = odom.pose.pose.position
            orientation = odom.pose.pose.orientation
            orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
            _, _, yaw = euler_from_quaternion(orientation_list)

            goal_angle = math.atan2(self.goal_y - self.current_position.y, self.goal_x - self.current_position.x)

            #print self.current_position.x + self.current_position.y
            direction_of_movement = goal_angle - yaw
            if direction_of_movement > pi:
                direction_of_movement -= 2 * pi

            elif direction_of_movement < -pi:
                direction_of_movement += 2 * pi

            self.direction_of_movement = round(direction_of_movement, 2)


    def getState(self, scan):
        scan_range = []
        direction_of_movement = self.direction_of_movement
        min_dist_for_collision = 0.13
        collision = False
        nr_of_samples = 23
        num_skipped_angles = int(len(scan.ranges) / nr_of_samples)
        for i in range(len(scan.ranges)):
            base = i % num_skipped_angles
            if base == 0:
                if scan.ranges[i] == float('Inf'):
                    scan_range.append(self.max_scan_value)
                elif scan.ranges[i] > self.max_scan_value:
                    scan_range.append(self.max_scan_value)
                elif np.isnan(scan.ranges[i]):
                    scan_range.append(float(0))
                else:
                    scan_value = float('%.2f' % scan.ranges[i])
                    scan_range.append(scan_value)

        closest_obstacle_distance = round(min(scan_range), 2)  # rounded to ndigits = 2
        closest_obstacle_angle = np.argmin(scan_range)

        # number of readings that are less than the minimum distance for collision
        possible_collision_readings = sum(i < min_dist_for_collision for i in scan_range)
        percentage_possible_collision_readings = float(possible_collision_readings) / len(scan_range) * 100

        # > 30% of scan values are showing a possible collision
        if percentage_possible_collision_readings > 30.0:
            collision = True

        if min_dist_for_collision > min(scan_range) > 0:  # check
            collision = True

        current_distance = round(math.hypot(self.goal_x - self.current_position.x, self.goal_y - self.current_position.y), 2)
        # if robot is 0.5m away from the goal, set the goal box to true - the goal has been reached
        if current_distance < 0.1:
            self.get_goalbox = True

        return scan_range + [direction_of_movement, current_distance, closest_obstacle_distance, closest_obstacle_angle], collision

    def getStateBackwards(self, scan):
        scan_range = []
        min_dist_for_collision = 0.6
        collision = False
        nr_of_samples = 23
        num_skipped_angles = int(len(scan.ranges) / nr_of_samples)
        for i in range(len(scan.ranges)):
            base = i % num_skipped_angles
            if base == 0:
                if scan.ranges[i] == float('Inf'):
                    scan_range.append(self.max_scan_value)
                elif scan.ranges[i] > self.max_scan_value:
                    scan_range.append(self.max_scan_value)
                elif np.isnan(scan.ranges[i]):
                    scan_range.append(float(0))
                else:
                    scan_value = float('%.2f' % scan.ranges[i])
                    scan_range.append(scan_value)

        closest_obstacle_distance = round(min(scan_range), 2)  # rounded to ndigits = 2
        closest_obstacle_angle = np.argmin(scan_range)

        # number of readings that are less than the minimum distance for collision
        possible_collision_readings = sum(i < min_dist_for_collision for i in scan_range)
        percentage_possible_collision_readings = float(possible_collision_readings) / len(scan_range) * 100

        # > 30% of scan values are showing a possible collision
        if percentage_possible_collision_readings > 30.0:
            collision = True

        if min_dist_for_collision > min(scan_range) > 0:  # check
            collision = True

        current_distance = round(math.hypot(self.goal_x - self.current_position.x, self.goal_y - self.current_position.y), 2)
        # if robot is 0.5m away from the goal, set the goal box to true - the goal has been reached
        if current_distance < 0.1:
            self.get_goalbox = True

        return collision

    def setReward(self, state, collision, action):
        yaw_reward = []

        #pdb.set_trace()
        #print("goal distance: ")
        #print(self.goal_distance)

        current_distance_from_goal = state[-3]
        #print("goal current_distance_from_goal: ")
        #print(current_distance_from_goal)
        direction_of_movement = state[-4]
        #print(" direction_of_movement: ")
        #print(direction_of_movement)
        for i in range(5):

            angle = pi / 4 + direction_of_movement + (pi / 8 * i)
            #print(" angle: ")
            #print(angle)
            #tr = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
            tr = math.sin(angle)
            #print("Fabss : ")
            #print(math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
            #print("tr : ")
            #print(tr)
            yaw_reward.append(tr)
            #print("yaw_reward : ")
            #print(yaw_reward)

            #print("yaw_reward : ")
            #print(yaw_reward)
        distance_rate = 2 ** (current_distance_from_goal / self.goal_distance)

        #print(" R thita ")
        #print(round(yaw_reward[action] * 5, 2))

        #print(" R from  Distance - distance_rate: ")
        #print(distance_rate)
        reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate)
        #print(" reward: ")
        #print(reward)

        if collision:
            rospy.loginfo("Collision!!")
            reward = -150
            cnt = 0
            while collision == True:
                cnt += 1
                collision = self.stepBackwards()
                #self.pub_cmd_vel.publish(Twist())

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
	    time.sleep(5)
            reward = 200
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True)
            self.goal_distance = self.getGoalDistace()
            self.get_goalbox = False

        return reward
        #print("Final reward : ")
        #print(reward)
    def step(self, action):
        max_angular_velocity = 1.5

        angular_velocity = ((self.action_size - 1) / 2 - action) * max_angular_velocity * 0.5

        move_velocity_cmd = Twist()
        move_velocity_cmd.linear.x = 0.15
        move_velocity_cmd.angular.z = angular_velocity
        self.pub_cmd_vel.publish(move_velocity_cmd)

        laser_scan_data = None
        while laser_scan_data is None:
            try:
                laser_scan_data = rospy.wait_for_message('scan', LaserScan, timeout=5)

            except:
                pass

        state, collision = self.getState(laser_scan_data)
        reward = self.setReward(state, collision, action)

        return np.asarray(state), reward, collision
    def stepBackwards(self):

        move_velocity_cmd = Twist()
        move_velocity_cmd.linear.x = 0.0
        move_velocity_cmd.angular.z = 1.5
        self.pub_cmd_vel.publish(move_velocity_cmd)

        laser_scan_data = None
        while laser_scan_data is None:
            try:
                laser_scan_data = rospy.wait_for_message('scan', LaserScan, timeout=5)

            except:
                pass
        collision = self.getStateBackwards(laser_scan_data)
        return collision
    def reset(self):
        # reset environment

        #os.system('rosservice call /gazebo/reset_world "{}"')

        # set up the odometry reset publisher
        #reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
        # reset odometry (these messages take a few iterations to get through)
        #timer = time()
        #while time() - timer < 0.25:
        #    reset_odom.publish(Empty())
        print("X:")
        print self.current_position.x
        print("Y:")
        print self.current_position.y
        laser_scan_data = None

        # make sure laser scan values exist
        while laser_scan_data is None:
            try:
                laser_scan_data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        # States last 4 elements are important -4 is heading, -3 is current distance, -2 is obstacle min range, and the -1 is obstacle angle.

        # if has reached a previous goal, set a new goal

        if self.initGoal:

            self.goal_x, self.goal_y = self.respawn_goal.getPosition()
            self.initGoal = False


        self.goal_distance = self.getGoalDistace()
        state, collision = self.getState(laser_scan_data)

        return np.asarray(state)
