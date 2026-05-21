import numpy as np

def target_constaint(target_x , target_y , l1 , l2):
    target_d = np.sqrt(target_x**2 + target_y**2)
    if target_d > l1+l2:
        scale = (l1+l2)/target_d
        target_x *=scale
        target_y *=scale
    return target_x , target_y
    