#!/usr/bin/env python


import rospy
import pyqtgraph as pg
import sys
import pickle
from std_msgs.msg import Float32MultiArray, Float32
from PyQt4.QtGui import *
from PyQt4.QtCore import *

class Window(QMainWindow):
    def __init__(self):
        super(Window, self).__init__()
        self.setWindowTitle("Result")
        self.setGeometry(50, 50, 600, 650)
        self.graph_sub = rospy.Subscriber('result', Float32MultiArray, self.data)
        self.ep = []
        self.data = []
        self.rewards = []
        self.x = []
        self.count = 1
        self.size_ep = 0
        load_data = False

        if load_data:
            self.ep, self.data = self.load_data()
            self.size_ep = len(self.ep)
        self.plot()

    def data(self, data):
        self.data.append(data.data[0])
        self.ep.append(self.size_ep + self.count)
        self.count += 1
        self.rewards.append(data.data[1])


    def plot(self):
        self.qValuePlt = pg.PlotWidget(self, title="Average of the Max Q-value")
        self.qValuePlt.move(0, 320)
        self.qValuePlt.resize(600, 300)
        self.timer1 = pg.QtCore.QTimer()
        self.timer1.timeout.connect(self.update)
        self.timer1.start(200)

        self.rewardsPlt = pg.PlotWidget(self, title="Total reward")
        self.rewardsPlt.move(0, 10)
        self.rewardsPlt.resize(600, 300)

        self.timer2 = pg.QtCore.QTimer()
        self.timer2.timeout.connect(self.update)
        self.timer2.start(100)

        self.show()

    def update(self):
        self.rewardsPlt.showGrid(x=True, y=True)
        self.qValuePlt.showGrid(x=True, y=True)
        self.rewardsPlt.plot(self.ep, self.data, pen=(255, 0, 0))
        self.qValuePlt.plot(self.ep, self.rewards, pen=(0, 255, 0))
        self.save_data([self.ep, self.data],[self.ep, self.rewards])
    def load_data(self):
        try:
            with open("graph.txt") as f:
                x, y = pickle.load(f)
        except:
            x, y = [], []
        return x, y

    def save_data(self, data, rewards):
        with open("graph.txt", "wb") as f:
            pickle.dump(data, f)
        with open("maxqgraph.txt", "wb") as f:
            pickle.dump(rewards, f)


def run():
        rospy.init_node('graph')
        app = QApplication(sys.argv)
        GUI = Window()
        sys.exit(app.exec_())

run()
