import os

import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
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


# Sequential:
# https://keras.io/getting-started/sequential-model-guide/
#
# load_model:
# You can then use keras.models.load_model(filepath) to reinstantiate your model.
# load_model will also take care of compiling the model using the saved training configuration (unless the model was never compiled in the first place).
# Link : https://keras.io/getting-started/faq/

from keras.models import Sequential, load_model
from keras.optimizers import RMSprop
from keras.layers.core import Dense, Dropout, Activation

EPISODES = 6000

class NeuralNetwork:
    def __init__(self, state_size, action_size):
        self.pub_result = rospy.Publisher('result', Float32MultiArray, queue_size=5)
        self.result = Float32MultiArray()

        self.dirPath = os.path.dirname(os.path.realpath(__file__))
        print(self.dirPath)
        #self.dirPath = "/home/turtlebot2i/src/turtlebot2i_deep_qlearning/turtlebot2i_deep_qlearning/dqn_refactored"
        #self.file_path = os.path.dirname(os.path.realpath(__file__)) + "/saved_weights/" + "model_"
        self.dirPath = self.dirPath.replace('turtlebot2i_deep_qlearning/dqn_refactored', 'turtlebot2i_deep_qlearning/save_model/model_')

        self.load_model = True
        self.load_episode = 5230

        self.state_size = state_size
        self.action_size = action_size
        self.episode_step = 6000
        self.target_update = 2000
        self.discount_factor = 0.99
        self.learning_rate = 0.00025
        self.epsilon = 0.0
        self.epsilon_decay = 1.0
        self.epsilon_min = 0.05
        self.batch_size = 64
        self.train_start = 64
        self.memory = deque(maxlen=1000000)

        # initialize a model
        self.model = self.buildModel()
        # initialize a target model
        self.target_model = self.buildModel()

        self.updateTargetModel()

        if self.load_model:
            self.model.set_weights(load_model(self.dirPath + str(self.load_episode) + ".h5").get_weights())

            # with open(self.dirPath + str(self.load_episode) + '.json') as outfile:
            #     param = json.load(outfile)
            #     self.epsilon = param.get('epsilon')
        # inorder to save the model
        # self.model.save("dqn_model.h5")

    def buildModel(self):
        # Keras Sequential model
        model = Sequential()
        dropout = 0.2
        # Dense is the basic form of a neural network layer and Input Layer of state size(4) and Hidden Layer with 64
        #  nodes kernel_initializer: Initializations define the way to set the initial random weights of Keras
        # layers. LeCun uniform initializer. Link https://keras.io/initializers/ It draws samples from a uniform
        # distribution within [-limit, limit] where limit is sqrt(3 / fan_in) where  fan_in is the number of input
        # units in the weight tensor. I guess the fan_id in our case is 64  so sqrt(3 / 64) = 0.21650635094
        model.add(Dense(64, input_shape=(self.state_size,), activation='relu', kernel_initializer='lecun_uniform'))
        # kernel_initializer sqrt(3 / 64) = 0.21650635094
        model.add(Dense(64, activation='relu', kernel_initializer='lecun_uniform'))
        model.add(Dropout(dropout))

        model.add(Dense(self.action_size, kernel_initializer='lecun_uniform'))
        model.add(Activation('linear'))
        # Compilation  Before training a model, you need to configure the learning process,
        # which is done via the compile method. It receives three arguments: - An optimizer - RMSprop is optimizer
        # and is usually a good choice for recurrent neural networks. rho is the "Gradient moving average [also
        # exponentially weighted average] decay factor and decay is the Learning rate decay over each update
        # Link: https://stats.stackexchange.com/questions/351409/difference-between-rho-and-decay-arguments-in-keras
        # rmsprop  A loss function: This is the objective that the model will try to minimize - A list of metrics:
        # this is for any classification problem

        model.compile(loss='mse', optimizer=RMSprop(lr=self.learning_rate, rho=0.9, epsilon=1e-06))
        # https://towardsdatascience.com/a-look-at-gradient-descent-and-rmsprop-optimizers-f77d483ef08b
        model.summary()

        return model

    # To make the agent perform well in mid-term and long-term, we need to take into account not only the immediate
    # rewards, but also the future rewards we are going to get.
    def getQvalue(self, reward, next_target, collision):
        if collision:
            return reward
        else:
            # In order to implement that, we will use gamma. In such a way, our DQN agent will learn to
            # maximize the discounted future reward on the given State.
            # In our case the discount factor is self.discount_factor
            # Furthermore, the np.amax is finding the max of the array.
            return reward + self.discount_factor * np.amax(next_target)

    def updateTargetModel(self):
        self.target_model.set_weights(self.model.get_weights())

    def getAction(self, state):
        if np.random.rand() <= self.epsilon:
            self.q_value = np.zeros(self.action_size)
            return random.randrange(self.action_size)
        else:
            q_value = self.model.predict(state.reshape(1, len(state)))
            self.q_value = q_value
            return np.argmax(q_value[0])

    # This function will simply store states, actions and resulting rewards into the memory:
    def appendMemory(self, state, action, reward, next_state, collision):
        self.memory.append((state, action, reward, next_state, collision))

    def trainModel(self, target=False):
        mini_batch = random.sample(self.memory, self.batch_size)
        X_batch = np.empty((0, self.state_size), dtype=np.float64)
        Y_batch = np.empty((0, self.action_size), dtype=np.float64)

        for i in range(self.batch_size):

            states = mini_batch[i][0]
            actions = mini_batch[i][1]
            rewards = mini_batch[i][2]
            next_states = mini_batch[i][3]
            collisions = mini_batch[i][4]

            q_value = self.model.predict(states.reshape(1, len(states)))
            self.q_value = q_value
            # After training, the model can now predict the output from unseen input. When you call the   predict()
            # function on the model, the model will predict the reward of the current State based on the data you
            # trained.
            if target:
                # the next_target contains 5 values
                next_target = self.target_model.predict(next_states.reshape(1, len(next_states)))
                # print(next_target)
            else:
                next_target = self.model.predict(next_states.reshape(1, len(next_states)))
                # print(next_target)
            next_q_value = self.getQvalue(rewards, next_target, collisions)
            # One of the most important steps in the learning process is to remember what we did in the past and how
            # the reward was bound to that action. Therefore, we need a list of previous experiences and observations
            #  to re-train the model with those previous experiences. We will store our experiences into an array
            # called memory and we will create aremember() function to append state, action, reward, and next state
            # to the array memory.
            X_batch = np.append(X_batch, np.array([states.copy()]), axis=0)
            Y_sample = q_value.copy()

            Y_sample[0][actions] = next_q_value
            Y_batch = np.append(Y_batch, np.array([Y_sample[0]]), axis=0)

            if collisions:
                X_batch = np.append(X_batch, np.array([next_states.copy()]), axis=0)
                Y_batch = np.append(Y_batch, np.array([[rewards] * self.action_size]), axis=0)
        # https://medium.com/@gtnjuvin/my-journey-into-deep-q-learning-with-keras-and-gym-3e779cc12762
        # In order to understand and predict our neural network, we have to feed it with inputs.
        # In order to do so, Keras provides the method fit(), which feeds input and output pairs to the model.
        # Then the model will train on the basis of those data to estimate the output based on the input.
        # This training process makes the neural network able to predict the reward value from a state.
        self.model.fit(X_batch, Y_batch, batch_size=self.batch_size, epochs=1, verbose=0)
