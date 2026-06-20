import numpy as np
from collision import check_collision_angles

class Node:
    def __init__(self, theta1, theta2):
        self.theta1 = theta1
        self.theta2 = theta2
        self.parent = None
        
        
def random_config():
    
    theta1 = np.random.uniform(-np.pi , np.pi)
    theta2 = np.random.uniform(-np.pi , np.pi)
        
    return theta1 , theta2
    
def distance(node, theta1, theta2):

    return np.sqrt(
        (node.theta1 - theta1)**2 +
        (node.theta2 - theta2)**2
    )
    
def nearest_node(tree, theta1, theta2):

    nearest = tree[0]

    min_distance = distance(
        nearest,
        theta1,
        theta2
    )

    for node in tree:

        d = distance(
            node,
            theta1,
            theta2
        )

        if d < min_distance:

            min_distance = d
            nearest = node

    return nearest

def steer(
    from_node,
    to_theta1,
    to_theta2,
    step_size=0.1
):

    dx = to_theta1 - from_node.theta1
    dy = to_theta2 - from_node.theta2

    length = np.sqrt(dx**2 + dy**2)

    if length < step_size:

        return Node(
            to_theta1,
            to_theta2
        )

    dx /= length
    dy /= length

    new_theta1 = (
        from_node.theta1
        + dx * step_size
    )

    new_theta2 = (
        from_node.theta2
        + dy * step_size
    )

    return Node(
        new_theta1,
        new_theta2
    )
    
    
def rrt(arm,goal_theta1,goal_theta2,obstacle_x,obstacle_y,obstacle_radius,ax,tree_lines,path_lines):
    start_node = Node(arm.theta1 , arm.theta2)
    tree = [start_node]
    
    goal_node = None
    
    for i in range(2000):

        # rand_theta1, rand_theta2 = random_config()
        if np.random.rand() < 0.2:
            rand_theta1 = goal_theta1
            rand_theta2 = goal_theta2
        else:
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