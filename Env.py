from abc import ABC, abstractmethod
import numpy as np
from AABB import Node
import pygame
from itertools import chain

# Set up the colors.
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
WHITE = (255, 255, 255)

pygame.init()
my_font = pygame.font.SysFont(None, 20)

# class EnvNode:
#     def __init__(self, node, g=inf, rhs=inf):
#         self.x = node.x
#         self.y = node.y
#         self.width = node.width
#         self.height = node.height
#         self.value = node.value
#         self.neighbors = node.neighbors
#         self.g = g
#         self.rhs = rhs
#         self.h = 0
#         self.in_queue = False
# 
#     def draw(self, window):
#         pygame.draw.rect(window, BLACK, (self.x - self.width / 2, self.y - self.height / 2,
#                                          self.width, self.height), 1 - self.value)
#         # for neighbor in self.neighbors:
#         #     pygame.draw.line(window, GREEN, (self.x, self.y), (neighbor.x, neighbor.y))
# 
#     def calculate_key(self):
#         return [min(self.g, self.rhs) + self.h, min(self.g, self.rhs)]
# 
#     def calculate_rhs(self):
#         return min([(cost(self, neighbor) + neighbor.g) for neighbor in self.neighbors])


def my_iter(iterable):
    if type(iterable[0]) == list:
        return chain.from_iterable(iterable)
    return iterable


class Environment(ABC):
    def __init__(self, x, y, env_width, env_height):
        self.root = Node(x, y, env_width, env_height)
        self.nodes = []
        self.start = None
        self.goal = None
        self.current = None

    @abstractmethod
    def update(self, obstacles):
        pass

    @abstractmethod
    def build_env(self, start, goal):
        pass

    def draw(self, window, mode="full"):
        if mode == 'full':
            # self.current.draw(window)
            for node in my_iter(self.nodes):
                # if node == self.start:
                #     start_text = my_font.render("Current", True, (0, 0, 0))
                #     start_rect = start_text.get_rect(center=tuple([node.x, node.y]))
                #     window.blit(start_text, start_rect)
                # if node == self.goal:
                #     goal_text = my_font.render("Goal", True, (0, 0, 0))
                #     goal_rect = goal_text.get_rect(center=tuple([node.x, node.y]))
                #     window.blit(goal_text, goal_rect)
                node.draw(window)
                for neighbor in node.neighbors:
                   pygame.draw.line(window, GREEN, (node.x, node.y), (neighbor.x, neighbor.y))
        if mode == 'boundary':
            self.root.draw(window)
        if mode == 'none':
            return


class QuadTreeEnvironment(Environment):
    def __init__(self, x, y, env_width, env_height):
        super().__init__(x, y, env_width, env_height)
        self.nodes = [self.root]

    def update(self, obstacles):
        self.root.update_percentage_and_split(obstacles)

    def build_env(self, start, goal):
        nodes = []
        self.add_start(start)
        self.add_goal(goal)
        for leaf in self.root.get_leaves():
            leaf.update_neighbors()
            nodes.append(leaf)
        self.nodes = nodes
        self.current = self.start

    def add_start(self, start):
        current = self.root
        while current.get_children():
            current = current.get_quadrant(start)
        self.start = current

    def add_goal(self, goal):
        current = self.root
        while current.get_children():
            current = current.get_quadrant(goal)
        self.goal = current

    # def clear(self):
    #     self.nodes = [self.root]
    #     self.start = None
    #     self.goal = None
    #     self.current = None
    # 
    # def solve(self):
    #     priority_queue = PriorityQueue()
    #     self.goal.rhs = 0
    #     priority_queue.insert(self.goal)
    #     compute_path(priority_queue, self)
    # 
    # def show_path(self):
    #     return show_path(self)


class GridEnvironment(Environment):
    def __init__(self, x, y, env_width, env_height, size=32):
        super().__init__(x, y, env_width, env_height)
        self.size = size
        self.cell_width = env_width / size
        self.cell_height = env_height / size
        self.left_pad = x - env_width / 2
        self.north_pad = y - env_height / 2
        self.nodes = [[Node(x + (2 * j - size + 1) * env_width / (2 * size),
                            y + (2 * i - size + 1) * env_height / (2 * size),
                            self.cell_width,
                            self.cell_height) for j in range(size)] for i in range(size)]


    def update(self, obstacles):
        for obstacle in obstacles:
            x1, x2, y1, y2 = obstacle.return_coordinate()
            lb_x, ub_x = int((x1 - self.left_pad) / self.cell_width), int((x2 - self.left_pad) / self.cell_width)
            lb_y, ub_y = int((y1 - self.north_pad) / self.cell_height), int((y2 - self.north_pad) / self.cell_height)
            for r in range(lb_y, ub_y + 1):
                for c in range(lb_x, ub_x + 1):
                    self.nodes[r][c].value = 1

    def build_env(self, start, goal):
        start_x, start_y = int((start[0] - self.left_pad) / self.cell_width), int((start[1] - self.north_pad) / self.cell_height)
        self.start = self.nodes[start_y][start_x]

        end_x, end_y = int((goal[0] - self.left_pad)/ self.cell_width), int((goal[1] - self.north_pad) / self.cell_height)
        self.goal = self.nodes[end_y][end_x]

        self.current = self.start

        for i in range(self.size):
            for j in range(self.size):
                node = self.nodes[i][j]
                if node.value == -1:
                    node.value = 0
                for dx, dy in ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)):
                    n_x, n_y = i + dx, j + dy
                    if 0 <= n_x < self.size and 0 <= n_y < self.size:
                        node.neighbors.append(self.nodes[n_x][n_y])


def compute_path(queue, graph):
    # while ((not queue.is_empty()) and compare_array(queue.top_key(), graph.current.calculate_key())) or \
    #         (graph.current.g != graph.current.rhs):
    while (queue and compare_array(queue[0].key, graph.current.calculate_key())) or (graph.current.g != graph.current.rhs):
        v = queue.pop(0)
        if v.g > v.rhs:
            v.g = v.rhs
            for u in v.neighbors:
                update_vertex(queue, graph, u)
        else:
            v.g = np.inf
            update_vertex(queue, graph, v)
            for u in v.neighbors:
                update_vertex(queue, graph, u)


def update_vertex(queue, graph, vertex):
    if vertex != graph.goal:
        vertex.calculate_rhs()
    # if vertex.in_queue:
    #     queue.remove(vertex)
    queue.discard(vertex)
    if vertex.g != vertex.rhs:
        # queue.insert(vertex)
        vertex.calculate_key()
        queue.add(vertex)


def compare_array(a, b):
    for i in range(max(len(a), len(b))):
        if i > len(b):
            return False
        if a[i] < b[i]:
            return True
        elif a[i] > b[i]:
            return False


def show_path(graph):
    path = [graph.current]
    current = graph.current
    while current != graph.goal:
        # cur_neighbors = current.neighbors
        # min_idx = np.argmin([v.g for v in cur_neighbors])
        current = min(current.neighbors, key=lambda x: x.g)
        path.append(current)
    return path

 
def draw_env_path(path, window, start, end, draw_robot=True):
    n = len(path)
    if n:
        # path = np.array([[p.x for p in path], [p.y for p in path]])
        if type(path[-1]) == tuple:
            pygame.draw.line(window, GREEN, (path[0].x, path[0].y), path[-1], 3)
        else:
            for i in range(n - 1):
                start_pos = (path[i].x, path[i].y) if i else start
                end_pos = (path[i + 1].x, path[i + 1].y) if i < n - 2 else end
                pygame.draw.line(window, GREEN, start_pos, end_pos, 3)
        if draw_robot:
            path[0].draw(window)
            if n > 1 and type(path[1]) != tuple:
                path[1].draw(window)


def draw_path(path, window, color):
    n = len(path)
    for i in range(n - 1):
        start_pos = (path[i][0], path[i][1])
        end_pos = (path[i + 1][0], path[i + 1][1])
        pygame.draw.line(window, color, start_pos, end_pos, 3)


def draw_target(window, target):
    target_img = pygame.image.load('flag.png')
    window.blit(target_img, target)


def draw_local_goal(window, local_goal):
    pygame.draw.circle(window, (255, 0, 0), local_goal, 2.5)

