import numpy as np
from collision import check_collision_angles

class Node:
    def __init__(self,theta1, theta2):
        self.theta1 = theta1
        self.theta2 = theta2
        self.parent = None
        self.g = float('inf')
        self.h = 0
        self.f = float('inf')
        
def get_neighbors(node, step_size = 0.3):
    neighbors =[]
    for dtheta1 in [-step_size, 0, step_size]:
        for dtheta2 in [-step_size, 0, step_size]:
            if dtheta1 == 0 and dtheta2 == 0:
                continue
            neighbor = Node(
                node.theta1 + dtheta1,
                node.theta2 + dtheta2
            )
            neighbors.append(neighbor)
    return neighbors
    
        
def heuristic(node , goal_theta1 , goal_theta2):
    return np.sqrt(
        (node.theta1 - goal_theta1)**2 + (node.theta2 - goal_theta2)**2
    )

def a_star(arm, goal_theta1, goal_theta2, obstacle_x, obstacle_y, obstacle_radius, ax, tree_lines, path_lines):
    start_node = Node(arm.theta1 , arm.theta2)
    start_node.g = 0
    start_node.h = heuristic(start_node , goal_theta1 , goal_theta2)
    start_node.f = start_node.h
    tree = [start_node]
    open_set = [start_node]
    closed_set = set()
    all_nodes = {}
    start_key = (round(start_node.theta1 , 3), round(start_node.theta2 , 3))
    all_nodes[start_key] = start_node
    goal_node = None
    
    while open_set:
        current_node = min(open_set , key = lambda n:n.f)
        open_set.remove(current_node)
        closed_set.add((round(current_node.theta1,3),round(current_node.theta2,3)))
        print("node: ",current_node.theta1 ," ",current_node.theta2)
        neighbors = get_neighbors(current_node)
        print("neigbors")
        for neighbor in neighbors:
            print(neighbor.theta1 , " ", neighbor.theta2)
            neighbor_key = (
            round(neighbor.theta1,3),
            round(neighbor.theta2,3)
            )
            if neighbor_key in closed_set:
                print("neighbor ",neighbor.theta1 , " ", neighbor.theta2 , " already exprored")
                continue
            if check_collision_angles(
                arm,
                neighbor.theta1,
                neighbor.theta2,
                obstacle_x,
                obstacle_y,
                obstacle_radius
            ):
                continue                
            tentative_g = current_node.g + heuristic(current_node , neighbor.theta1 , neighbor.theta2)
            if neighbor_key not in all_nodes:
                all_nodes[neighbor_key] = neighbor
                
            neighbor = all_nodes[neighbor_key]
            if tentative_g < neighbor.g:
                neighbor.g = tentative_g
                neighbor.h = heuristic(neighbor , goal_theta1 , goal_theta2)
                neighbor.f = neighbor.g + neighbor.h
                neighbor.parent = current_node 
                if neighbor not in open_set:
                    open_set.append(neighbor)
            tree.append(neighbor)  
            if heuristic(neighbor , goal_theta1 , goal_theta2) < 0.3 :
                goal_node = neighbor
                break
        if goal_node is not None:
            break
                
            

    print("out of the loop")       
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
            path.append((current.theta1 , current.theta2))
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
    

    