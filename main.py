import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider

from arm import RoboticArm2D
from config import *
from utils import target_constaint
from ik import analytical_ik
from collision import trajectory_collides, check_collision

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
    color='red',
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

    waypoint_plot.set_data([], [])

    try:

        target_solutions = analytical_ik(
            target_x,
            target_y,
            arm.l1,
            arm.l2
        )

        found = False

        for theta1, theta2 in target_solutions:

            arm.plan_trajectory(
                theta1,
                theta2,
                steps=100
            )

            direct_collision = trajectory_collides(
                arm,
                arm.trajectory,
                obstacle_x,
                obstacle_y,
                obstacle_radius
            )

            if not direct_collision:

                arm.current_waypoint = 0
                found = True
                break

            dx = target_x - obstacle_x
            dy = target_y - obstacle_y

            length = np.sqrt(dx**2 + dy**2)

            if length == 0:
                continue

            dx /= length
            dy /= length

            directions = [
                ( dy,  dx),
                (-dy, -dx),
                ( dy, -dx),
                (-dy,  dx)
            ]

            for radius in [0.6, 0.8, 1.0, 1.2]:

                if found:
                    break

                for perp_x, perp_y in directions:

                    waypoint_x = obstacle_x + perp_x * radius
                    waypoint_y = obstacle_y + perp_y * radius

                    waypoint_plot.set_data(
                        [waypoint_x],
                        [waypoint_y]
                    )

                    try:

                        waypoint_solutions = analytical_ik(
                            waypoint_x,
                            waypoint_y,
                            arm.l1,
                            arm.l2
                        )

                        for w_theta1, w_theta2 in waypoint_solutions:

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
                                theta1,
                                theta2,
                                steps=50
                            )

                            second_part = arm.trajectory.copy()

                            candidate_trajectory = (
                                first_part +
                                second_part
                            )

                            arm.theta1 = old_theta1
                            arm.theta2 = old_theta2

                            collides = trajectory_collides(
                                arm,
                                candidate_trajectory,
                                obstacle_x,
                                obstacle_y,
                                obstacle_radius
                            )

                            if not collides:

                                arm.trajectory = candidate_trajectory
                                arm.current_waypoint = 0

                                found = True
                                break

                        if found:
                            break

                    except:
                        pass

                if found:
                    break

            if found:
                break

    except Exception as e:
        print(e)

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