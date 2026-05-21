import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider

from arm import RoboticArm2D
from config import *
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

# joint1_path, = ax.plot([] , [] , "--" , lw=2)
# end_path, = ax.plot([] , [] , "r-" , lw=2)

# joint1_x_history = []
# joint1_y_history = []

# end_x_history = []
# end_y_history = []

def on_click(event):
    global target_x , target_y
    
    if event.xdata is None or event.ydata is None:
        return
    target_x = event.xdata
    target_y = event.ydata
    print(f"Target: ({target_x:.2f}, {target_y:.2f})")
    
fig.canvas.mpl_connect('button_press_event', on_click)

def update(frame):

    try:
        theta1 , theta2 = analytical_ik(target_x,target_y, arm.l1 , arm.l2)
        arm.set_angles(theta1, theta2)
        
    except:
        pass
    
    x, y = arm.forward_kinematics()

    line.set_data(x, y)
    
    target_plot.set_data([target_x], [target_y])
     # Extract points
    # x1, y1 = x[1], y[1]
    # x2, y2 = x[2], y[2]

    # Store history
    # joint1_x_history.append(x1)
    # joint1_y_history.append(y1)

    # end_x_history.append(x2)
    # end_y_history.append(y2)

    # Draw trajectory curves
    # joint1_path.set_data(joint1_x_history, joint1_y_history)
    # end_path.set_data(end_x_history, end_y_history)

    return line,target_plot
#   joint1_path, end_path

ani = FuncAnimation(fig, update, interval=1000/FPS)

plt.show()