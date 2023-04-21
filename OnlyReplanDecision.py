import numpy as np


class OnlyReplanDecision:
    def __init__(self):
        self.goal = None
        self.obstacles_list_before = None
        self.obstacles_list_after = None

    def update(self, obstacles_list_before, obstacles_list_after, goal):
        self.obstacles_list_before = obstacles_list_before
        self.obstacles_list_after = obstacles_list_after
        self.goal = goal
    def decisionMaking(self, rb):
        decision = "No"
        for i in range(len(self.obstacles_list_before)):
            x1 = self.obstacles_list_before[i].x
            y1 = self.obstacles_list_before[i].y

            rb_next = rb.nextPosition(self.goal)

            distance = np.sqrt((rb.pos[0] - x1)*(rb.pos[0] - x1) + (rb.pos[1] - y1)*(rb.pos[1] - y1))
            phi = angle(rb_next[0] - rb.pos[0], rb_next[1] - rb.pos[1], x1 - rb.pos[0], y1 - rb.pos[1])
            if distance < rb.r and phi <= np.pi / 2:
                decision = "Replan"
        return decision


def angle(x1, y1, x2, y2):
    return np.arccos((x1 * x2 + y1 * y2) / (np.sqrt(x1 * x1 + y1 * y1) * np.sqrt(x2 * x2 + y2 * y2) +1e-6))