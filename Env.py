import numpy as np
from AABB import Node
import pygame

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


class Environment:
    def __init__(self, x, y, env_width, env_height):
        self.root = Node(x, y, env_width, env_height)
        self.nodes = [self.root]
        self.start = None
        self.goal = None
        self.current = None

    def update(self, obstacles):
        for obstacle in obstacles:
            self.root.update_percentage_and_split(obstacle)

    def build_env(self, start, goal):
        nodes = []
        self.add_start(start)
        self.add_goal(goal)
        for leaf in self.root.get_leaves():
            leaf.init()
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

    def draw(self, window, mode='full'):
        if mode == 'full':
            for node in self.nodes:
                # if node == self.start:
                #     start_text = my_font.render("Current", True, (0, 0, 0))
                #     start_rect = start_text.get_rect(center=tuple([node.x, node.y]))
                #     window.blit(start_text, start_rect)
                if node == self.goal:
                    goal_text = my_font.render("Goal", True, (0, 0, 0))
                    goal_rect = goal_text.get_rect(center=tuple([node.x, node.y]))
                    window.blit(goal_text, goal_rect)
                node.draw(window)
                # for neighbor in node.neighbors:
                #    pygame.draw.line(window, GREEN, (node.x, node.y), (neighbor.x, neighbor.y))
        if mode == 'boundary':
            self.root.draw(window)
        if mode == 'none':
            return
    
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


def compute_path(queue, graph):
    # while ((not queue.is_empty()) and compare_array(queue.top_key(), graph.current.calculate_key())) or \
    #         (graph.current.g != graph.current.rhs):
    while (queue and compare_array(queue[0].calculate_key(), graph.current.calculate_key())) or \
            (graph.current.g != graph.current.rhs):
        # v = queue.pop()
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
        # if queue.is_empty():
        if not queue:
            break


def update_vertex(queue, graph, vertex):
    if vertex != graph.goal:
        vertex.calculate_rhs()
    # if vertex.in_queue:
    #     queue.remove(vertex)
    queue.discard(vertex)
    if vertex.g != vertex.rhs:
        # queue.insert(vertex)
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
        cur_neighbors = current.neighbors
        min_idx = np.argmin([v.g for v in cur_neighbors])
        current = cur_neighbors[min_idx]
        path.append(current)
    return path

 
def draw_env_path(path, window, draw_robot=True):
    n = len(path)
    if n:
        # path = np.array([[p.x for p in path], [p.y for p in path]])
        if type(path[-1]) == tuple:
            pygame.draw.line(window, GREEN, (path[0].x, path[0].y), path[-1], 3)
        else:
            for i in range(n - 1):
                start_pos = (path[i].x, path[i].y)
                end_pos = (path[i + 1].x, path[i + 1].y)
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

