import rospy
import os
import json

import numpy as np
import random
import time
import sys

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

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
    start_time = time.time()

    for episode in range(neural_network.load_episode + 1, EPISODES):
        done = False
        # reset the environment first
        state = environment.reset()
        score = 0

        for step in range(neural_network.episode_step):
            action = neural_network.getAction(state)
            # make a move
            next_state, reward, done = environment.step(action)

            neural_network.appendMemory(state, action, reward, next_state, done)

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
            if episode % 10 == 0:
                neural_network.model.save(neural_network.file_path + str(episode) + '.h5')
                with open(neural_network.file_path + str(episode) + '.json', 'w') as outfile:
                    json.dump(parameter_dictionary, outfile)

            if step >= 500:
                rospy.loginfo("Time out! 500 steps complete!")
                done = True

            if done:
                result.data = [score, np.max(neural_network.q_value)]
                publish_result.publish(result)
                neural_network.updateTargetModel()
                scores.append(score)
                episodes.append(episode)
                # time for logging
                minute, second = divmod(int(time.time() - start_time), 60)
                hour, minute = divmod(minute, 60)
                rospy.loginfo('Ep: %d score: %.2f memory: %d epsilon: %.2f time: %d:%02d:%02d',
                              episode, score, len(neural_network.memory), neural_network.epsilon, hour, minute, second)
                parameter_key = ['epsilon']
                parameter_value = [neural_network.epsilon]
                parameter_dictionary = dict(zip(parameter_key, parameter_value))
                break

            global_step += 1

            if global_step % neural_network.target_update == 0:
                rospy.loginfo("UPDATE TARGET NETWORK")

        if neural_network.epsilon > neural_network.epsilon_min:
            neural_network.epsilon *= neural_network.epsilon_decay
