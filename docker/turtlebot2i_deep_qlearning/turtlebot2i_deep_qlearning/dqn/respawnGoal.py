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

        self.f = open(self.modelPath, 'r')
        self.model = self.f.read()
        self.stage = 2
        self.goal_position = Pose()
        self.init_goal_x = 0.6
        self.init_goal_y = 0.0
        self.goal_position.position.x = self.init_goal_x
        self.goal_position.position.y = self.init_goal_y
        self.modelName = 'goal'

        self.obstacle_1 = -1.0, -1.0
        self.obstacle_2 = -1.0, 1.0
        self.obstacle_3 = 1.0, -1.0
        self.obstacle_4 = 1.0, 1.0
        self.human_1 = 1.74723, -1.88126
        self.human_2 = -2.21807, 2.24004

        self.last_goal_x = self.init_goal_x
        self.last_goal_y = self.init_goal_y
        self.last_index = 0
        self.sub_model = rospy.Subscriber('gazebo/model_states', ModelStates, self.checkModel)
        self.check_model = False
        self.index = 0

    def checkModel(self, model):
        self.check_model = False
        for i in range(len(model.name)):
            if model.name[i] == "goal":
                self.check_model = True

    def respawnModel(self):
        while True:
            if not self.check_model:
                rospy.wait_for_service('gazebo/spawn_sdf_model')
                spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
                spawn_model_prox(self.modelName, self.model, 'robotos_name_space', self.goal_position, "world")
                rospy.loginfo("Goal position : %.1f, %.1f", self.goal_position.position.x,
                              self.goal_position.position.y)
                break
            else:
                #print("Goal Model did not spawn")
                pass

    def deleteModel(self):
        while True:
            if self.check_model:
                rospy.wait_for_service('gazebo/delete_model')
                del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
                del_model_prox(self.modelName)
                break
            else:
                pass


    def getPosition(self, position_check=False, delete=False):
        if delete:
            self.deleteModel()

        while position_check:
            goal_x = random.randrange(-26, 27) / 10.0  # (-12, 13)
            goal_y = random.randrange(-26, 27) / 10.0  # (-12, 13)
            if abs(goal_x - self.obstacle_1[0]) <= 0.4 and abs(goal_y - self.obstacle_1[1]) <= 0.4:
                position_check = True
            elif abs(goal_x - self.obstacle_2[0]) <= 0.4 and abs(goal_y - self.obstacle_2[1]) <= 0.4:
                position_check = True
            elif abs(goal_x - self.obstacle_3[0]) <= 0.4 and abs(goal_y - self.obstacle_3[1]) <= 0.4:
                position_check = True
            elif abs(goal_x - self.obstacle_4[0]) <= 0.4 and abs(goal_y - self.obstacle_4[1]) <= 0.4:
                position_check = True
            elif abs(goal_x - self.human_1[0]) <= 0.4 and abs(goal_y - self.human_1[1]) <= 0.4:
                position_check = True
            elif abs(goal_x - self.human_2[0]) <= 0.4 and abs(goal_y - self.human_2[1]) <= 0.4:
                position_check = True
            elif abs(goal_x - 0.0) <= 0.4 and abs(goal_y - 0.0) <= 0.4:
                position_check = True
            else:
                position_check = False

            if abs(goal_x - self.last_goal_x) < 1 and abs(goal_y - self.last_goal_y) < 1:
                position_check = True

            self.goal_position.position.x = goal_x
            self.goal_position.position.y = goal_y

        time.sleep(0.5)

        self.respawnModel()
        # Setting up the last goal position
        self.last_goal_x = self.goal_position.position.x
        self.last_goal_y = self.goal_position.position.y

        return self.goal_position.position.x, self.goal_position.position.y
