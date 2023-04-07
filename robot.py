import numpy as np
import pygame
from Env import compute_path, show_path, update_vertex
from DecisionMaking import fuzzyDecisionMaking


class Robot:
    def __init__(self, start, v=10, r=40):
        self.pos = start
        self.v = v
        self.r = r

    # def move(self, goal, obs):
    #     obstacles = self.detect(obs)
    #     self.pos = tuple(PSO(50, 50, self.pos, goal, obstacles=obstacles))

    def draw(self, window):
        pygame.draw.circle(window, (255, 0, 0), self.pos, 2, 0)
        
    def reach(self, goal, epsilon=8):
        robotX, robotY = self.pos
        goalX, goalY = goal
        return ((robotX - goalX) ** 2 + (robotY - goalY) ** 2) <= epsilon ** 2

    def enter(self, node):
        robotX, robotY = self.pos
        nodeX, nodeY, node_size = node.x, node.y, node.width
        return np.abs(robotX - nodeX) <= node_size / 2 and np.abs(robotY - nodeY) <= node_size / 2

    def updatePath(self, obstacles_list, priority_queue, env, threshold=1e-3):
        changes = []
        obstacles = self.detect(obstacles_list)
        for n in env.current.neighbors:
            percentage = 0
            for obstacle in obstacles:
                percentage += n.get_intersect_percentage(obstacle)
                if percentage >= threshold:
                    if n.value != 1:
                        changes.append(n)
                        n.value = 1
                    break
            if percentage < threshold:
                if n.value:
                    changes.append(n)
                n.value = 0
        for change in changes:
            update_vertex(priority_queue, env, change)
            for n in change.neighbors:
                update_vertex(priority_queue, env, n)
        compute_path(priority_queue, env)
        new_path = show_path(env)
        return new_path

    def detect(self, obstacles_list):
        obstacles = []
        for obstacle in obstacles_list:
            x1, x2, y1, y2 = obstacle.return_coordinate()
            top_left_x = max(x1, self.pos[0] - self.r)
            top_left_y = max(y1, self.pos[1] - self.r)
            bottom_right_x = min(x2, self.pos[0] + self.r)
            bottom_right_y = min(y2, self.pos[1] + self.r)
            dx = max(0, bottom_right_x - top_left_x)
            dy = max(0, bottom_right_y - top_left_y)
            if dx and dy:
                # x = (bottom_right_x + top_left_x) / 2
                # y = (bottom_right_y + top_left_y) / 2
                obstacles.append(obstacle)
        return obstacles

    def nextPosition(self, goal):
        return goal
        # return PSO(30, 25, self.pos, goal)

    def decisionMaking(self, obstacles_list_before, obstacles_list_after, goal):
        decision = "No"
        for i in range(len(obstacles_list_before)):
            x1 = obstacles_list_before[i].x
            y1 = obstacles_list_before[i].y
            x2 = obstacles_list_after[i].x
            y2 = obstacles_list_after[i].y
            if x1 == x2 and y1 == y2: continue
            x1, y1 = min(obstacles_list_before[i].get_corners(),
                         key=lambda x: (self.pos[0] - x[0]) ** 2 + (self.pos[1] - x[1]) ** 2)
            x2, y2 = min(obstacles_list_after[i].get_corners(),
                         key=lambda x: (self.pos[0] - x[0]) ** 2 + (self.pos[1] - x[1]) ** 2)
            distance = np.sqrt((self.pos[0] - x1)*(self.pos[0] - x1) + (self.pos[1] - y1)*(self.pos[1] - y1))
            if distance < self.r:
                distance_next = np.sqrt((self.pos[0] - x2) * (self.pos[0] - x2) + (self.pos[1] - y2) * (self.pos[1] - y2))
                rb_next = self.nextPosition(goal)
                phi = angle(rb_next[0] - self.pos[0], rb_next[1] - self.pos[1], x1 - self.pos[0], y1 - self.pos[1])
                phi_next = angle(rb_next[0] - self.pos[0], rb_next[1] - self.pos[1], x2 - self.pos[0], y2 - self.pos[1])
                decision_temp = fuzzyDecisionMaking(phi, phi_next, distance, distance_next)
                if decision_temp == "Replan":
                    return decision_temp
                elif decision_temp == "Stop":
                    decision = decision_temp
        return decision


def angle(x1, y1, x2, y2):
    return np.arccos((x1 * x2 + y1 * y2) / (np.sqrt(x1 * x1 + y1 * y1) * np.sqrt(x2 * x2 + y2 * y2) +1e-6))

