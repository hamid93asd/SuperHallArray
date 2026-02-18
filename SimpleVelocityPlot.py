import matplotlib.pyplot as plt
import numpy as np

DIRECTORY = '/Users/jacobdebry/Documents/HallArray/Pico/log/VelocityTestString'
FILE = 'devttyusbmodemPICO1_2026_02_17.15.27.06.743.txt'

data = np.loadtxt(f"{DIRECTORY}/{FILE}", delimiter=",")

c1, c2, c3, c4 = data.T

plt.plot(c1[:])
plt.plot(c2[:])
plt.show()



