import copy
import sys
import pygame
import time
import numpy as np
from pygame.locals import *
from sortedcontainers import SortedList
from Env import QuadTreeEnvironment, GridEnvironment, compute_path, show_path, draw_path, draw_target, draw_env_path
from robot import Robot
from Spline import makeSpline, drawSpline
from Obstacles import *


maps = {"trap": trap, "maze1": maze1, "maze2": maze2, "room1": room1, "room2": room2, "room3": room3, "dense1": dense1,
        "dense2": dense2, "dense3": dense3, "test": test, "fake_room": fake_room, "trap2": trap2, "trap3": trap3}
map = input("Enter map: ")
# env_width = int(input("Enter width: "))
# env_height = int(input("Enter height: "))
env_width = env_height = 512
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

#Frequency of global replan
PATIENCE = 30

# Initialization
begin = (64, 500)
end = (470, 180)
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
# local_path = []
spl = None
targets = 0
robot = Robot(begin)
start_time = end_time = None

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
                with open("Obstacles.py", 'a') as f:
                    f.write(",\n".join([o.__str__() for o in obstacles_list]))
                if map in maps:
                    obstacles_list = maps[map]
                start_time = time.time()
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
                new_obstacle = Obstacle((mx+new_mx)/2, (my+new_my)/2, abs(new_mx-mx), abs(new_my-my), isStatic, np.random.randn(2) * 2)
                obstacles_list.append(new_obstacle)
                drawing = False
        if event.type == KEYDOWN:
            if event.key == pygame.K_z and pygame.key.get_mods() & pygame.KMOD_CTRL:
                if obstacles_list and not done:
                    obstacles_list.pop()
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
        obstacles_list_before = copy.deepcopy(obstacles_list)
        for obstacle in obstacles_list:
            obstacle.move()
            obstacle.draw(screen)
        obstacles_list_after = obstacles_list
        # if env:
        #     changed, newPath = robot.updatePath(obstacles_list, priority_queue, env)
        #     if changed:
        #         print("Changed")
        #         path = newPath
        #         spl = makeSpline(robot.pos, path)
        #         local_goal = tuple(spl[:, 499])
        #         targets = 0

        if patience:
            robotX, robotY = robot.pos
            if (len(past_path) == 0) or robot.pos != past_path[-1]:
                past_path.append(robot.pos)
            decision = robot.decisionMaking(obstacles_list_before, obstacles_list_after, local_goal)
            # print(decision)
            if decision == "Replan":
                path = robot.updatePath(obstacles_list, priority_queue, env)
                # print("Changed")
                spl = makeSpline(robot.pos, path, end)
                local_goal = tuple(spl[:, 200])
                targets = 1
            if decision != "Stop":
                # if len(path) > 2 and env.current.value > 0:
                #     # print("PSO")
                #     robot.move(local_goal, obstacles_list)
                # else:
                robot.pos = robot.nextPosition(local_goal)

            # p = robot.updatePath(obstacles_list, priority_queue, env)
            # if p:
            #     path = p
            #     spl = makeSpline(robot.pos, path, end)
            #     local_goal = tuple(spl[:, 200])
            #     targets = 1
            # robot.pos = robot.nextPosition(local_goal)

            # robot.move(local_goal)
            # local_path.append(robot.pos)
            if robot.reach(local_goal):
                if local_goal == end:
                    finished = True
                elif len(path) > 2 and robot.enter(path[1]):
                    path.pop(0)
                    env.current = path[0]
                targets += 1
                if targets * 200 < spl.shape[1]:
                    local_goal = tuple(spl[:, targets * 200])
                else:
                    local_goal = end
            patience += 1
            if patience == PATIENCE:
                patience = 0
        else:
            #Quadtree
            env = QuadTreeEnvironment(LEFT_PAD + env_width / 2, NORTH_PAD + env_height / 2, env_width, env_height)

            #Grid
            # env = GridEnvironment(LEFT_PAD + env_width / 2, NORTH_PAD + env_height / 2, env_width, env_height)
            env.update(obstacles_list)
            env.build_env(robot.pos, end)
            priority_queue = SortedList(key=lambda x: x.key)
            env.goal.rhs = 0
            # priority_queue.insert(env.goal)
            env.goal.calculate_key()
            priority_queue.add(env.goal)
            compute_path(priority_queue, env)
            path = show_path(env)
            spl = makeSpline(robot.pos, path, end)
            targets = 1
            if targets * 200 < spl.shape[1]:
                local_goal = tuple(spl[:, targets * 200])
            else:
                local_goal = end
            # if len(path) <= 2:
            #     local_goal = end
            # local_path = []
            patience += 1
        draw_path(past_path, screen, YELLOW)
        # draw_path(local_path, screen, BLUE)
        # draw_env_path(path, screen, robot.pos, end, draw_robot=False)
        drawSpline(spl, screen)
        # draw_local_goal(screen, local_goal)
        env.draw(screen, mode="boundary")
        draw_target(screen, (end[0] - 10, end[1] - 64))
        robot.draw(screen)
        if robot.reach(end):
            finished = True
        time.sleep(0.1)
    pygame.display.update()

end_time = time.time()

def angle(p1, p2, p3):
    l = np.sqrt(((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) * ((p3[0] - p2[0]) ** 2 + (p3[1] - p2[1]) ** 2))
    a = max(((p1[0] - p2[0]) * (p3[0] - p2[0]) + (p1[1] - p2[1]) * (p3[1] - p2[1])) / l, -1)
    return np.pi - np.arccos(a)


with open("comparison", "a") as f:
    d = 0
    for i in range(1, len(past_path)):
        d += np.sqrt(np.sum(np.square(np.array(past_path[i-1]) - np.array(past_path[i]))))
    count = smooth = 0
    for i in range(1, len(past_path) - 1):
        count += 1
        smooth += angle(past_path[i - 1], past_path[i], past_path[i + 1])
    # Write distance, smoothness and time of an execution to an output file
    # f.write(map + ' ' + str(round(d, 4)) + ' ' + str(round((smooth / count) * 180 / np.pi, 4)) +
    #         ' ' + str(round(end_time - start_time, 4)) + '\n')
    f.write(f"{map}: {round(d, 4)} {round((smooth / count) * 180 / np.pi, 4)} {round(end_time - start_time, 4)} \n")
