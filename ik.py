import numpy as np

def analytical_ik(x, y, l1, l2):

    d = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    
    d = np.clip(d,-1.0 , 1.0)
    theta2_down = np.arccos(d)
    theta2_up = -np.arccos(d)
    
    def solve_theta1(theta2):
        
        theta1 = np.arctan2(y, x) - np.arctan2(
            l2 * np.sin(theta2),
            l1 + l2 * np.cos(theta2)
        )
        return theta1
    
    theta1_down = solve_theta1(theta2_down)
    theta1_up = solve_theta1(theta2_up)
 
    return (
        (theta1_up, theta2_up),
        (theta1_down, theta2_down)
    )