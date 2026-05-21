import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from arm import RoboticArm2D
from config import *

arm = RoboticArm2D(LINK1_LENGTH, LINK2_LENGTH)


fig, ax = plt.subplots()

ax.set_xlim(-WINDOW_SIZE, WINDOW_SIZE)
ax.set_ylim(-WINDOW_SIZE, WINDOW_SIZE)
ax.set_aspect('equal')
ax.grid()

line, = ax.plot([], [], 'o-', lw=4)

joint1_path, = ax.plot([] , [] , "--" , lw=2)
end_path, = ax.plot([] , [] , "r-" , lw=2)

joint1_x_history = []
joint1_y_history = []

end_x_history = []
end_y_history = []

def update(frame):

    theta1 = PI * np.sin(frame * 0.02)
    theta2 = PI * np.sin(frame * 0.05)
    arm.set_angles(theta1, theta2)

    x, y = arm.forward_kinematics()

    line.set_data(x, y)
     # Extract points
    x1, y1 = x[1], y[1]
    x2, y2 = x[2], y[2]

    # Store history
    joint1_x_history.append(x1)
    joint1_y_history.append(y1)

    end_x_history.append(x2)
    end_y_history.append(y2)

    # Draw trajectory curves
    joint1_path.set_data(joint1_x_history, joint1_y_history)
    end_path.set_data(end_x_history, end_y_history)

    return line, joint1_path, end_path

ani = FuncAnimation(fig, update, interval=1000/FPS)

plt.show()