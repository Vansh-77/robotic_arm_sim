import pygame
import numpy as np

from arm import RoboticArm2D
from ik import analytical_ik
from config import *
from utils import target_constaint

pygame.init()

WIDTH = 800
HEIGHT = 800

screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

CENTER_X = WIDTH // 2
CENTER_Y = HEIGHT // 2

SCALE = 150

arm = RoboticArm2D(1.0, 1.0)

target_x = 1.5
target_y = 0.5

trail = []

speed = 0.03

def world_to_screen(x, y):

    screen_x = CENTER_X + x * SCALE
    screen_y = CENTER_Y - y * SCALE

    return int(screen_x), int(screen_y)

running = True

while running:

    dt = clock.tick(FPS)

    for event in pygame.event.get():

        if event.type == pygame.QUIT:
            running = False

    keys = pygame.key.get_pressed()

    if keys[pygame.K_w]:
        target_y += speed

    if keys[pygame.K_s]:
        target_y -= speed

    if keys[pygame.K_a]:
        target_x -= speed

    if keys[pygame.K_d]:
        target_x += speed

    target_x, target_y = target_constaint(
        target_x,
        target_y,
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

        theta1, theta2 = solutions[0]

        arm.set_angles(theta1, theta2)

    except:
        pass

    x, y = arm.forward_kinematics()

    trail.append((x[-1], y[-1]))

    if len(trail) > 3000:
        trail.pop(0)

    screen.fill((30, 30, 30))

    for i in range(len(trail) - 1):

        pygame.draw.line(
            screen,
            (255, 0, 0),
            world_to_screen(
                trail[i][0],
                trail[i][1]
            ),
            world_to_screen(
                trail[i + 1][0],
                trail[i + 1][1]
            ),
            2
        )

    pygame.draw.line(
        screen,
        (0, 150, 255),
        world_to_screen(x[0], y[0]),
        world_to_screen(x[1], y[1]),
        6
    )

    pygame.draw.line(
        screen,
        (0, 255, 150),
        world_to_screen(x[1], y[1]),
        world_to_screen(x[2], y[2]),
        6
    )

    pygame.draw.circle(
        screen,
        (255, 255, 255),
        world_to_screen(x[0], y[0]),
        8
    )

    pygame.draw.circle(
        screen,
        (255, 255, 255),
        world_to_screen(x[1], y[1]),
        8
    )

    pygame.draw.circle(
        screen,
        (255, 255, 255),
        world_to_screen(x[2], y[2]),
        8
    )

    pygame.draw.circle(
        screen,
        (0, 255, 0),
        world_to_screen(target_x, target_y),
        8
    )

    pygame.display.flip()

pygame.quit()