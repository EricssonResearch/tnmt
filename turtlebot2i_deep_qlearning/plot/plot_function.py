import matplotlib.pyplot as plt
import math
import numpy as np
def funct(angle):
    return 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
x = np.arange(-2*math.pi,2*math.pi,0.1)
y = []
sine = []
for i in x:
    y.append(funct(i))
    sine.append(math.sin(i))
plt.plot(x,y)
plt.plot(x,sine)
plt.ylabel('function')
plt.xlabel('x')
plt.title("y")
plt.show()
