import sys
import pygame
import time
from pygame.locals import *
from sortedcontainers import SortedList
from AABB import Obstacle
from Env import Environment, compute_path, show_path, draw_env_path, draw_path, draw_target, draw_local_goal
from robot import Robot
from Spline import makeSpline, drawSpline


env_width = int(input("Enter width: "))
env_height = int(input("Enter height: "))
# Pygame setup
NORTH_PAD, SOUTH_PAD, LEFT_PAD, RIGHT_PAD = (int(env_height * 0.06), int(env_height * 0.16),
                                             int(env_width * 0.06), int(env_width * 0.06))
SCREEN_WIDTH = env_width + LEFT_PAD + RIGHT_PAD
SCREEN_HEIGHT = env_height + NORTH_PAD + SOUTH_PAD
pygame.init()
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)
pygame.display.set_caption("Quadtree Simulation")
my_font = pygame.font.SysFont(None, SOUTH_PAD // 4)

# Set up the colors.
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
PATIENCE = 30

# Initialization
begin = (40, 500)
end = (470, 140)
mx, my, new_mx, new_my = None, None, None, None
drawing = False
done = False
finished = False
isStatic = True
obstacles_list = []
pause = False
patience = 0
env = None
path = None
past_path = []
local_path = []
spl = None
targets = 0
robot = Robot(begin)

while not finished:
    screen.fill(WHITE)

    # Button
    button1 = pygame.draw.rect(screen, BLACK, (LEFT_PAD + int(env_width * 0.1), NORTH_PAD * 2 + env_height,
                                               int(env_width * 0.2), int(SOUTH_PAD * 0.4)), 4)
    button1_text = my_font.render("Start", True, (0, 0, 0))
    button1_rect = button1_text.get_rect(center=button1.center)
    screen.blit(button1_text, button1_rect)

    button2 = pygame.draw.rect(screen, BLACK, (LEFT_PAD + int(env_width * 0.7), NORTH_PAD * 2 + env_height,
                                               int(env_width * 0.2), int(SOUTH_PAD * 0.4)), 4)
    button2_text = my_font.render("Pause", True, (0, 0, 0))
    button2_rect = button2_text.get_rect(center=button2.center)
    screen.blit(button2_text, button2_rect)

    button3 = pygame.draw.rect(screen, BLACK, (LEFT_PAD + int(env_width * 0.4), NORTH_PAD * 2 + env_height,
                                               int(env_width * 0.2), int(SOUTH_PAD * 0.4)), 4)
    button3_text = my_font.render("Static", True, (0, 0, 0))
    button3_rect = button3_text.get_rect(center=button3.center)
    screen.blit(button3_text, button3_rect)

    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            sys.exit()
        if event.type == MOUSEBUTTONDOWN:
            mouse_x, mouse_y = event.pos
            if button1.collidepoint(mouse_x, mouse_y):
                done = True
            elif button2.collidepoint(mouse_x, mouse_y):
                pause = not pause
            elif button3.collidepoint(mouse_x, mouse_y):
                isStatic = not isStatic
                if isStatic:
                    print("Static")
                else:
                    print("Dynamic")
            else:
                mx, my = mouse_x, mouse_y
                drawing = True
                done = False
        if event.type == MOUSEBUTTONUP:
            if drawing:
                new_mx, new_my = event.pos
                new_obstacle = Obstacle((mx+new_mx)/2, (my+new_my)/2, abs(new_mx-mx), abs(new_my-my), static=isStatic)
                obstacles_list.append(new_obstacle)
                drawing = False
    if drawing:
        new_mx, new_my = pygame.mouse.get_pos()
        pygame.draw.rect(screen, BLACK, (min(mx, new_mx), min(my, new_my), abs(new_mx - mx), abs(new_my - my)))
    if not done:
        for obstacle in obstacles_list:
            obstacle.draw(screen)
        pygame.draw.rect(screen, BLACK, (LEFT_PAD, NORTH_PAD, env_width, env_height), 3)
    elif pause:
        continue
    else:
        obstacles_list_before = obstacles_list
        for obstacle in obstacles_list:
            obstacle.move()
            obstacle.draw(screen)
        obstacles_list_after = obstacles_list
        if env:
            changed, newPath = robot.updatePath(obstacles_list, priority_queue, env)
            if changed:
                print("Changed")
                path = newPath
                spl = makeSpline(robot.pos, path)
                local_goal = tuple(spl[:, 499])
                targets = 0
                # spline = makeSpline(robot.pos, path, screen)
                # local_goal = tuple(spline[500])
                # local_goal = compute_local_path(path[1], end)

            print(robot.decisionMaking(obstacles_list_before, obstacles_list_after, local_goal))

        if patience:
            robotX, robotY = robot.pos
            past_path.append(robot.pos)
            # new_x, new_y = PSO(50, 25, (robotX, robotY), local_goal)
            # robot.move((new_x, new_y))
            # local_path.append((new_x, new_y))

            robot.move(local_goal)
            local_path.append(robot.pos)

            if robot.reach(local_goal):
                if local_goal == end:
                    finished = True
                else:
                    path.pop(0)
                    env.current = path[0]
                    targets += 1
                    if len(path) > 2:
                        local_goal = tuple(spl[:, targets * 1000 + 499])
                    else:
                        path[-1] = end
                        local_goal = end
            patience += 1
            if patience == PATIENCE:
                patience = 0
        else:
            env = Environment(LEFT_PAD + env_width / 2, NORTH_PAD + env_height / 2, env_width, env_height)
            env.update(obstacles_list)
            env.build_env(robot.pos, end)
            # priority_queue = PriorityQueue()
            priority_queue = SortedList(key=lambda x: x.calculate_key())
            env.goal.rhs = 0
            # priority_queue.insert(env.goal)
            priority_queue.add(env.goal)
            compute_path(priority_queue, env)
            path = show_path(env)
            spl = makeSpline(robot.pos, path)
            targets = 0
            if len(path) > 2:
                local_goal = tuple(spl[:, targets * 1000 + 499])
            else:
                local_goal = end
            # if len(path) <= 2:
            #     local_goal = end
            local_path = []
            patience += 1
        draw_path(past_path, screen, YELLOW)
        draw_path(local_path, screen, BLUE)
        # draw_env_path(path, screen)
        drawSpline(spl, screen)
        draw_local_goal(screen, local_goal)
        env.draw(screen)
        draw_target(screen, (end[0] - 10, end[1] - 64))
        robot.draw(screen)
        if robot.reach(end):
            finished = True
        time.sleep(1.0)
    pygame.display.update()
