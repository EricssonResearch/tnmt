#!/usr/bin/env python


import rospy
import random
import time
import os
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
import pdb;

class Respawn():
    def __init__(self):
        self.modelPath = os.path.dirname(os.path.realpath(__file__)) + "/goal_square/goal_box/model.sdf"
        # self.modelPath = os.path.dirname(os.path.realpath(__file__))
        # self.modelPath = self.modelPath.replace('turtlebot3_machine_learning/turtlebot3_dqn/src/turtlebot3_dqn',
        #                                        'turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_square/goal_box/model_old_1.sdf')
        self.f = open(self.modelPath, 'r')
        self.model = self.f.read()
        self.stage = 2
        self.goal_position = Pose()


        self.modelName = 'goal'
        # self.obstacle_1 = 1.5, 1.5
        # self.obstacle_2 = 1.5, -1.5
        # self.obstacle_3 = -1.5, 1.5
        # self.obstacle_4 = -1.5, -1.5

        self.wall_0 = 0 , 2.925000
        self.wall_2 = -2.925000 , 0
        self.wall_3 = 0 , -2.925000
        self.wall_4 = 2.925000 , 0



        self.goal_a_x = 2.02
        self.goal_a_y = 1.66
        self.goal_b_x = 1.5
        self.goal_b_y = -0.28
        self.goal_c_x = 1.04
        self.goal_c_y = -2.04
        self.which_goal = 0
        self.nr_of_goals = 3
        self.init_goal_x = self.goal_a_x
        self.init_goal_y = self.goal_a_y
        self.last_goal_x = self.init_goal_x
        self.last_goal_y = self.init_goal_y
        self.goal_position.position.x = self.init_goal_x
        self.goal_position.position.y = self.init_goal_y
        self.last_index = 0
        self.sub_model = rospy.Subscriber('gazebo/model_states', ModelStates, self.checkModel)
        self.check_model = False
        self.index = 0

    def checkModel(self, model):
        self.check_model = False
        for i in range(len(model.name)):
            if model.name[i] == "goal":
                self.check_model = True


    def getPosition(self, position_check=False):

        self.which_goal += 1
        self.which_goal = self.which_goal  % self.nr_of_goals
        if self.which_goal == 0:
            # respawn to goal a
            self.goal_position.position.x = self.goal_a_x
            self.goal_position.position.y = self.goal_a_y
        elif self.which_goal == 1:
            # respawn to goal b
            self.goal_position.position.x = self.goal_b_x
            self.goal_position.position.y = self.goal_b_y
        elif self.which_goal == 2:
            # respawn to goal b
            self.goal_position.position.x = self.goal_c_x
            self.goal_position.position.y = self.goal_c_y
        time.sleep(0.5)

        # Setting up the last goal position
        self.last_goal_x = self.goal_position.position.x
        self.last_goal_y = self.goal_position.position.y

        return self.goal_position.position.x, self.goal_position.position.y
