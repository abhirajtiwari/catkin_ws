import numpy as np

def cardiod(a, thetas):
    return a*(1+np.cos(thetas))

def nomask(a, thetas):
    return a
