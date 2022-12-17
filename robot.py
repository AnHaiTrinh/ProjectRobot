import pygame
from Env import compute_path, show_path, update_vertex
from PSO import PSO


class Robot:
    def __init__(self, start, v=10, r=30):
        self.pos = start
        self.v = v
        self.r = r

    def move(self, goal):
        # obstacles = self.detect(obs)
        self.pos = PSO(30, 25, self.pos, goal)

    def draw(self, window):
        pygame.draw.circle(window, (255, 0, 0), self.pos, 4, 0)
        
    def reach(self, goal, epsilon=4):
        robotX, robotY = self.pos
        goalX, goalY = goal
        return ((robotX - goalX) ** 2 + (robotY - goalY) ** 2) <= epsilon


    def updatePath(self, obstacles_list, priority_queue, env):
        changes = []
        obstacles = self.detect(obstacles_list)
        print(obstacles)
        for n in env.current.neighbors:
            if n.value == 0:
                percentage = 0
                for obstacle in obstacles:
                    percentage += n.get_intersect_percentage(obstacle)
                    if percentage >= 5e-4:
                        n.value = 1
                        changes.append(n)
                        break
                break

        if not changes:
            return False, None
        for change in changes:
            update_vertex(priority_queue, env, change)
            for n in change.neighbors:
                update_vertex(priority_queue, env, n)

        # for vertex in self.curNode.neighbors:
        #     if vertex.value == 1:
        #         cs = []
        #         for v in vertex.neighbors:
        #             cs.append(v)
        #         for c in cs:
        #             update_vertex(priority_queue, env, c)
        #             for n in c.neighbors:
        #                 update_vertex(priority_queue, env, n)
        compute_path(priority_queue, env)
        new_path = show_path(env)
        return True, new_path

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

