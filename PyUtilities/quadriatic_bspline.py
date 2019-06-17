import os
import numpy as np
import matplotlib.pyplot as plt

# Node at left boundary 0.0 ~ 1.5
def head0(u):
    if (u >= 0.0):
        if (u < 0.5):
            return 1.0 - 4.0/3.0 * u*u
        elif (u <= 1.5):
            return 2.0/3.0 * u*u - 2.0 * u + 1.5
    return 0.0

# One node away from left boundary -1.0 ~ 1.5
def head1(u):
    if (u >= -1.0):
        if (u < -0.5):
            return 4.0/3.0 * (u+1.0)*(u+1.0)
        elif (u < 0.5):
            return -7.0/6.0 * u*u + u/6.0 + 17.0/24.0
        elif (u <= 1.5):
            return (3.0-2.0*u)*(3.0-2.0*u) / 8.0
    return 0.0

# Internal nodes -1.5 ~ 1.5
def middle(u):
    if (u >= -1.5):
        if (u < -0.5):
            return (2.0*u+3.0)*(2.0*u+3.0) / 8.0
        elif (u < 0.5):
            return 0.75 - u*u
        elif (u <= 1.5):
            return (3.0-2.0*u)*(3.0-2.0*u) / 8.0
    return 0.0

# One node away from right boundary -1.5 ~ 1.0
def tail1(u):
    if (u >= -1.5):
        if (u < -0.5):
            return (3.0+2.0*u)*(3.0+2.0*u) / 8.0
        elif (u < 0.5):
            return -7.0/6.0 * u*u - u / 6.0 + 17.0/24.0
        elif (u <= 1.0):
            return 4.0/3.0 * (1.0-u)*(1.0-u)
    return 0.0

# nodes at right boundary -1.5 ~ 0.0
def tail0(u):
    if (u >= -1.5):
        if (u < -0.5):
            return 2.0/3.0 * u*u + 2.0 * u + 1.5
        elif (u <= 0.0):
            return 1.0 - 4.0/3.0 * u*u
    return 0.0
    
def cal_func(func, x1, x2, u1, u2, num):
    x = np.zeros(num+1)
    y = np.zeros(num+1)
    x_div = (x2 - x1) / num
    u_div = (u2 - u1) / num
    for i in range(num+1):
        x[i] = x1 + x_div * i
        y[i] = func(u1 + u_div * i)
    return (x, y)

if __name__ == "__main__":
    fig = plt.figure()
    plot1 = fig.subplots(1, 1)
    plot1.set_xlim([0.0, 5.0])
    plot1.set_ylim([0.0, 1.0])
    # head0
    x, y = cal_func(head0, 0.0, 1.5, 0.0, 1.5, 100)
    plot1.plot(x, y)
    # head1
    x, y = cal_func(head1, 0.0, 2.5, -1.0, 1.5, 100)
    plot1.plot(x, y)
    # middle1
    x, y = cal_func(middle, 0.5, 3.5, -1.5, 1.5, 100)
    plot1.plot(x, y)
    # middle2
    x, y = cal_func(middle, 1.5, 4.5, -1.5, 1.5, 100)
    plot1.plot(x, y)
    # tail1
    x, y = cal_func(tail1, 2.5, 5.0, -1.5, 1.0, 100)
    plot1.plot(x, y)
    # tail0
    x, y = cal_func(tail0, 3.5, 5.0, -1.5, 0.0, 100)
    plot1.plot(x, y)
    plt.show()
