import matplotlib.pyplot as plt
import numpy as np

from scipy import signal


def sq_wave(t):
    f = 0.5
    const = 2 * np.pi * f * t
    delta_angle = (4/np.pi) * (np.sin(const) + (1/3)*np.sin(3*const) + (1/5)*np.sin(5*const)+ (1/7)*np.sin(7*const))
    # delta_angle = np.sin(np.pi*f*t)
    return delta_angle

y = np.arange(0,200,0.1)
x =[]
for i in y:
    # x.append(sq_wave(i))
    x.append(signal.square(2 * np.pi * 0.1* i, duty=0.5))
plt.plot(y,x)
plt.show()
