import json


class QtableSaver:

    def saveQtable(self, qtable):
        with open('qtable.json', 'w') as file:
            json.dump(qtable, file)

    def openQtable(self):
        with open('qtable.json', 'r') as fp:
            saved_qtable = json.load(fp)
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
# q = QtableSaver()
# QtableSaver.saveQtable(q, qtable)
# qt = QtableSaver.openQtable(q)
# print(qt["state2"])
