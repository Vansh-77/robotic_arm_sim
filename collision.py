import numpy as np

def point_to_segment_distance(px, py, x1, y1, x2, y2):

    line_dx = x2 - x1
    line_dy = y2 - y1

    length_sq = line_dx**2 + line_dy**2

    if length_sq == 0:
        return np.sqrt((px - x1)**2 + (py - y1)**2)

    t = (
        (px - x1) * line_dx +
        (py - y1) * line_dy
    ) / length_sq

    t = np.clip(t, 0, 1)

    nearest_x = x1 + t * line_dx
    nearest_y = y1 + t * line_dy

    return np.sqrt(
        (px - nearest_x)**2 +
        (py - nearest_y)**2
    )
    
def check_collision(x, y, obstacle_x, obstacle_y, obstacle_radius):

    d1 = point_to_segment_distance(
        obstacle_x,
        obstacle_y,
        x[0], y[0],
        x[1], y[1]
    )

    d2 = point_to_segment_distance(
        obstacle_x,
        obstacle_y,
        x[1], y[1],
        x[2], y[2]
    )

    return (
        d1 < obstacle_radius or
        d2 < obstacle_radius
    )
    
def check_collision_angles(
    arm,
    theta1,
    theta2,
    obstacle_x,
    obstacle_y,
    obstacle_radius
):

    old_theta1 = arm.theta1
    old_theta2 = arm.theta2

    arm.theta1 = theta1
    arm.theta2 = theta2

    x, y = arm.forward_kinematics()

    d1 = point_to_segment_distance(
        obstacle_x,
        obstacle_y,
        x[0], y[0],
        x[1], y[1]
    )

    d2 = point_to_segment_distance(
        obstacle_x,
        obstacle_y,
        x[1], y[1],
        x[2], y[2]
    )

    arm.theta1 = old_theta1
    arm.theta2 = old_theta2

    return (
        d1 < obstacle_radius or
        d2 < obstacle_radius
    )
def trajectory_collides(
    arm,
    trajectory,
    obstacle_x,
    obstacle_y,
    obstacle_radius
):

    old_theta1 = arm.theta1
    old_theta2 = arm.theta2

    for theta1, theta2 in trajectory:

        arm.theta1 = theta1
        arm.theta2 = theta2

        x, y = arm.forward_kinematics()

        d1 = point_to_segment_distance(
            obstacle_x,
            obstacle_y,
            x[0], y[0],
            x[1], y[1]
        )

        d2 = point_to_segment_distance(
            obstacle_x,
            obstacle_y,
            x[1], y[1],
            x[2], y[2]
        )

        if (
            d1 < obstacle_radius
            or
            d2 < obstacle_radius
        ):

            arm.theta1 = old_theta1
            arm.theta2 = old_theta2

            return True

    arm.theta1 = old_theta1
    arm.theta2 = old_theta2

    return False