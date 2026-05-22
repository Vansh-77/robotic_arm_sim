import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider

from arm import RoboticArm2D
from config import *
from utils import *
from ik import analytical_ik
from collision import *

l1 = LINK1_LENGTH
l2 = LINK2_LENGTH

arm = RoboticArm2D(l1, l2)

target_x = 1.5
target_y = 1.0

fig, ax = plt.subplots()

plt.subplots_adjust(bottom=0.15)

l1_slider = Slider(
    ax=plt.axes([0.25, 0.00, 0.65, 0.03]),
    label='L1',
    valmin=0.1,
    valmax=2.0,
    valinit=l1
)

l2_slider = Slider(
    ax=plt.axes([0.25, 0.05, 0.65, 0.03]),
    label='L2',
    valmin=0.1,
    valmax=2.0,
    valinit=l2
)

def update_sliders(val):
    arm.l1 = l1_slider.val
    arm.l2 = l2_slider.val

l1_slider.on_changed(update_sliders)
l2_slider.on_changed(update_sliders)

ax.set_xlim(-WINDOW_SIZE, WINDOW_SIZE)
ax.set_ylim(-WINDOW_SIZE, WINDOW_SIZE)

ax.set_aspect('equal')
ax.grid()

line, = ax.plot([], [], 'o-', lw=4)

target_plot, = ax.plot([], [], 'rx', markersize=12)

waypoint_plot, = ax.plot([], [], 'go', markersize=10)

obstacle_x = 1
obstacle_y = 1
obstacle_radius = 0.3

obstacle = plt.Circle(
    (obstacle_x, obstacle_y),
    obstacle_radius,
    color="red",
    alpha=0.5
)

ax.add_patch(obstacle)

def on_click(event):

    global target_x, target_y

    if event.xdata is None or event.ydata is None:
        return

    target_x, target_y = target_constaint(
        event.xdata,
        event.ydata,
        arm.l1,
        arm.l2
    )

    try:

        solutions = analytical_ik(
            target_x,
            target_y,
            arm.l1,
            arm.l2
        )

        best_solution = None

        for theta1, theta2 in solutions:

            collision = check_collision_angles(
                arm,
                theta1,
                theta2,
                obstacle_x,
                obstacle_y,
                obstacle_radius
            )

            if not collision:
                best_solution = (theta1, theta2)
                break

        if best_solution is None:
            return

        arm.plan_trajectory(
            best_solution[0],
            best_solution[1]
        )

        collision = trajectory_collides(
            arm,
            arm.trajectory,
            obstacle_x,
            obstacle_y,
            obstacle_radius
        )

        if collision:
            

            dx = target_x - obstacle_x
            dy = target_y - obstacle_y

            length = np.sqrt(dx**2 + dy**2)

            dx /= length
            dy /= length

            perp_x = dy
            perp_y = -dx

            waypoint_x = obstacle_x + perp_x * 0.8
            waypoint_y = obstacle_y + perp_y * 0.8

            waypoint_plot.set_data(
                [waypoint_x],
                [waypoint_y]
            )

            waypoint_solutions = analytical_ik(
                waypoint_x,
                waypoint_y,
                arm.l1,
                arm.l2
            )

            w_theta1, w_theta2 = waypoint_solutions[0]

            old_theta1 = arm.theta1
            old_theta2 = arm.theta2

            arm.plan_trajectory(
                w_theta1,
                w_theta2,
                steps=50
            )

            first_part = arm.trajectory.copy()

            arm.theta1 = w_theta1
            arm.theta2 = w_theta2

            arm.plan_trajectory(
                best_solution[0],
                best_solution[1],
                steps=50
            )

            second_part = arm.trajectory.copy()

            arm.theta1 = old_theta1
            arm.theta2 = old_theta2

            arm.trajectory = (
                first_part +
                second_part
            )

            arm.current_waypoint = 0

        else:

            waypoint_plot.set_data([], [])

    except:
        pass

fig.canvas.mpl_connect(
    'button_press_event',
    on_click
)

def update(frame):

    arm.update_motion()

    x, y = arm.forward_kinematics()

    line.set_data(x, y)

    target_plot.set_data(
        [target_x],
        [target_y]
    )

    collision = check_collision(
        x,
        y,
        obstacle_x,
        obstacle_y,
        obstacle_radius
    )

    if collision:
        line.set_color('red')
    else:
        line.set_color('blue')

    return (
        line,
        target_plot,
        waypoint_plot
    )

ani = FuncAnimation(
    fig,
    update,
    interval=1000 / FPS
)

plt.show()