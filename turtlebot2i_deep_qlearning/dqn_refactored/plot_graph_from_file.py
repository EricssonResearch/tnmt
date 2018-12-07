import numpy as np
import matplotlib.pyplot as plt
import pickle
import sys

def load_data_rewards():
    try:
        with open("graph.txt") as f:
            x, y = pickle.load(f)
    except:
        x, y = [], []
    return x, y
def load_data_qmaxval():
    try:
        with open("maxqgraph.txt") as f:
            x, y = pickle.load(f)
    except:
        x, y = [], []
    return x, y
ep, data = load_data_rewards()
ep, qmaxval = load_data_qmaxval()
size_ep = len(ep)
plt.subplot(2, 1, 1)
plt.plot(ep,data)
plt.ylabel('Episode rewards')
plt.xlabel('Episode')
plt.title("Rewards")
plt.subplot(2, 1, 2)
plt.plot(ep,qmaxval)
plt.ylabel('Average Q value')
plt.xlabel('Episode')
plt.show()
