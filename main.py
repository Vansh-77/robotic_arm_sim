import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider

from arm import RoboticArm2D
from config import *
from utils import target_constaint
from ik import analytical_ik
from collision import *
from rrt import *

l1 = LINK1_LENGTH
l2 = LINK2_LENGTH

arm = RoboticArm2D(l1, l2)

target_x = l1 + l2
target_y = 0

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

tree_lines = []
path_lines = []

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
    target_solutions = analytical_ik(
    target_x,
    target_y,
    arm.l1,
    arm.l2
    )
    goal_theta1 , goal_theta2 = target_solutions[0]
    for theta1 , theta2 in target_solutions:
        if not check_collision_angles(
            arm,
            theta1,
            theta2,
            obstacle_x,
            obstacle_y,
            obstacle_radius
        ):
            goal_theta1 , goal_theta2 = theta1 , theta2
            break
            
    
    start_node = Node(arm.theta1 , arm.theta2)
    tree = [start_node]
    
    goal_node = None
    
    for i in range(2000):

        rand_theta1, rand_theta2 = random_config()

        nearest = nearest_node(
            tree,
            rand_theta1,
            rand_theta2
        )

        new_node = steer(
            nearest,
            rand_theta1,
            rand_theta2,
            step_size=0.15
        )

        collision = check_collision_angles(
            arm,
            new_node.theta1,
            new_node.theta2,
            obstacle_x,
            obstacle_y,
            obstacle_radius
        )

        if collision:
            continue

        new_node.parent = nearest

        tree.append(new_node)

        goal_distance = np.sqrt(

        (new_node.theta1 - goal_theta1)**2 +

        (new_node.theta2 - goal_theta2)**2
        )

        if goal_distance < 0.2:

            goal_node = new_node
            break
    
    for l in tree_lines:
        l.remove()

    tree_lines.clear()

    for l in path_lines:
        l.remove()

    path_lines.clear() 

    for node in tree:

        if node.parent is None:
            continue

        old_theta1 = arm.theta1
        old_theta2 = arm.theta2

        arm.theta1 = node.theta1
        arm.theta2 = node.theta2

        x1, y1 = arm.get_end_effector()

        arm.theta1 = node.parent.theta1
        arm.theta2 = node.parent.theta2

        x2, y2 = arm.get_end_effector()

        arm.theta1 = old_theta1
        arm.theta2 = old_theta2

        tree_line, = ax.plot(
            [x1, x2],
            [y1, y2],
            color='gray',
            alpha=0.3
        )
        tree_lines.append(tree_line)
    
    if goal_node is not None:

        path = []

        current = goal_node

        while current is not None:

            path.append(
                (
                current.theta1,
                current.theta2
                )
            )

            current = current.parent

        path.reverse()
        path_x = []
        path_y = []

        for theta1, theta2 in path:

            old_theta1 = arm.theta1
            old_theta2 = arm.theta2

            arm.theta1 = theta1
            arm.theta2 = theta2

            x, y = arm.get_end_effector()

            path_x.append(x)
            path_y.append(y)

            arm.theta1 = old_theta1
            arm.theta2 = old_theta2

        path_line, = ax.plot(
            path_x,
            path_y,
            color='green',
            linewidth=3
        )
        path_lines.append(path_line)

        arm.trajectory = path
        arm.current_waypoint = 0
    

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