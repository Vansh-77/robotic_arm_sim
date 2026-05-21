import numpy as np

def analytical_ik(x, y, l1, l2):

    d = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)

    theta2 = np.arccos(d)

    theta1 = np.arctan2(y, x) - np.arctan2(
        l2 * np.sin(theta2),
        l1 + l2 * np.cos(theta2)
    )

    return theta1, theta2