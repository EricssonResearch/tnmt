import pickle
import os
from pathlib import Path
import pdb


class QtableSave:
    def saveQtable(self, qtable):
        path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(path, "qtable.pickle"), 'wb') as file:
            pickle.dump(qtable, file, protocol=pickle.HIGHEST_PROTOCOL)

    def openQtable(self):
        path = os.path.dirname(os.path.abspath(__file__))
        path1 = os.path.join(path, "qtable.pickle")
        with open(path1, 'rb') as file:
            saved_qtable = pickle.load(file)
            return saved_qtable


# usage: qtable is the dict of interest
# pass it to the saveQtable method which takes the instance of the object and the q table as the parameters
# to open q table, pass the instance of the qtable object only
# sample implementation shown below

# qtable = {
#     "state1": 10120,
#     "state2": 11121,
#     "state3": 13331
# }
#q = QtableSave()
# Qtable.saveQtable(q, qtable)
#qt = QtableSave.openQtable(q)
#print(qt)
#print(q.openQtable())
