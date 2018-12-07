import rospy

# The OS module in Python provides a way of using operating system dependent functionality.
#
# The functions that the OS module provides allows you to interface with the
# underlying operating system that Python is running on be that Windows Mac or Linux
#  - https://www.pythonforbeginners.com/os/pythons-os-module
import os
import json

# NumPy is the fundamental package for scientific computing with Python. It contains among other things:
# a powerful N-dimensional array object
# sophisticated (broadcasting) functions
# tools for integrating C/C++ and Fortran code
# useful linear algebra, Fourier transform, and random number capabilities
import numpy as np
import random
import time
import sys

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

# This module implements specialized container datatypes providing alternatives to Pythons general purpose built in
# containers, dict, list, set, and tuple.

from collections import deque
from std_msgs.msg import Float32MultiArray


from NeuralNetwork import NeuralNetwork

from Environment import Environment
import pdb
EPISODES = 6000

if __name__ == '__main__':
    rospy.init_node('turtlebot2i_deep_qlearning')
    publish_result = rospy.Publisher('result', Float32MultiArray, queue_size=5)
    publish_action = rospy.Publisher('get_action', Float32MultiArray, queue_size=5)

    # store the action taken and the result
    action_data = Float32MultiArray()
    result = Float32MultiArray()

    state_space_size = 28
    num_of_actions = 5

    parameter_dictionary = None

    environment = Environment(num_of_actions)

    neural_network = NeuralNetwork(state_space_size, num_of_actions)
    scores, episodes = [], []
    global_step = 0
    step = 0
    start_time = time.time()
    #for episode in range(neural_network.load_episode + 1, EPISODES):
    collision = False
    # reset the environment first
    #pdb.set_trace()
    state = environment.reset()
    score = 0
    while True:
    #for step in range(neural_network.episode_step):
        action = neural_network.getAction(state)
        # make a move
        next_state, reward, collision = environment.step(action)

        neural_network.appendMemory(state, action, reward, next_state, collision)

        if len(neural_network.memory) >= neural_network.train_start:
            if global_step <= neural_network.target_update:
                neural_network.trainModel()
            else:
                neural_network.trainModel(True)

        score += reward
        state = next_state

        action_data.data = [action, score, reward]
        publish_action.publish(action_data)

        # save after every 10 episodes
        # if global_step % 10000 == 0:
        #     neural_network.model.save(neural_network.dirPath + str(global_step) + '.h5')
        #     with open(neural_network.dirPath + str(global_step) + '.json', 'w') as outfile:
        #         json.dump(parameter_dictionary, outfile)

        #if step >= 500:


        # if collision:
        #     result.data = [score, np.max(neural_network.q_value)]
        #     publish_result.publish(result)
        #     neural_network.updateTargetModel()
        #     scores.append(score)
        #     episodes.append(episode)
        #     # time for logging
        #     minute, second = divmod(int(time.time() - start_time), 60)
        #     hour, minute = divmod(minute, 60)
        #     rospy.loginfo('Ep: %d score: %.2f memory: %d epsilon: %.2f time: %d:%02d:%02d',
        #                   episode, score, len(neural_network.memory), neural_network.epsilon, hour, minute, second)
        #     parameter_key = ['epsilon']
        #     parameter_value = [neural_network.epsilon]
        #     parameter_dictionary = dict(zip(parameter_key, parameter_value))
        #     break

        global_step += 1
        step += 1
        # if global_step % neural_network.target_update == 0:
        #     rospy.loginfo("UPDATE TARGET NETWORK")

    if neural_network.epsilon > neural_network.epsilon_min:
        neural_network.epsilon *= neural_network.epsilon_decay
