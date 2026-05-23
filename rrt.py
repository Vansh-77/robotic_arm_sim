import numpy as np

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