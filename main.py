import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider

from arm import RoboticArm2D
from config import *
from utils import * 
from ik import analytical_ik

l1 = LINK1_LENGTH
l2 = LINK2_LENGTH


arm = RoboticArm2D(l1, l2)

target_x = 1.5
target_y = 1.0

fig, ax = plt.subplots()

l1_slider = Slider(
    ax = plt.axes([0.25, 0, 0.65, 0.03]),
    label='L1',
    valmin=0.1,
    valmax=2.0,
    valinit=l1
)
l2_slider = Slider(
    ax = plt.axes([0.25 , 0.05 , 0.65 , 0.03]),
    label='L2',
    valmin=0.1,
    valmax=2.0,
    valinit=l2
)
def update(val):
    arm.l1 = l1_slider.val
    arm.l2 = l2_slider.val

l1_slider.on_changed(update)
l2_slider.on_changed(update)

ax.set_xlim(-WINDOW_SIZE, WINDOW_SIZE)
ax.set_ylim(-WINDOW_SIZE, WINDOW_SIZE)
ax.set_aspect('equal')
ax.grid()

line, = ax.plot([], [], 'o-', lw=4)
target_plot, = ax.plot([], [], 'rx', markersize=12)


def on_click(event):
    global target_x , target_y
    
    if event.xdata is None or event.ydata is None:
        return

    target_x , target_y = target_constaint(event.xdata,event.ydata,arm.l1 , arm.l2)
    
    print(f"Target: ({target_x:.2f}, {target_y:.2f})")
    
fig.canvas.mpl_connect('button_press_event', on_click)

def update(frame):

    try:
        theta1 , theta2 = analytical_ik(target_x,target_y, arm.l1 , arm.l2)
        arm.target_theta1 = theta1
        arm.target_theta2 = theta2
        
    except:
        pass
    
    arm.update_motion()
    x, y = arm.forward_kinematics()

    line.set_data(x, y)
    
    target_plot.set_data([target_x], [target_y])

    return line,target_plot


ani = FuncAnimation(fig, update, interval=1000/FPS)

plt.show()