import numpy as np

ANGLE_F = 15
ANGLE_DF = 35
ANGLE_DS = 55
ANGLE_S = 75

DELTAD_A = 2.5
DELTAD_UA = 1.5
DELTAD_UC = -1.5
DELTAD_C = -2.5

DELTAPHI_A = 10
DELTAPHI_LAA = 7
DELTAPHI_LAU = 5
DELTAPHI_ULA = 3
DELTAPHI_ULC = -3
DELTAPHI_LCU = -5
DELTAPHI_LCC = -7
DELTAPHI_C = -10


class FuzzyDecisionMaking:
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
            x2 = self.obstacles_list_after[i].x
            y2 = self.obstacles_list_after[i].y
            if x1 == x2 and y1 == y2: continue
            x1, y1 = min(self.obstacles_list_before[i].get_corners(),
                         key=lambda x: (rb.pos[0] - x[0]) ** 2 + (rb.pos[1] - x[1]) ** 2)
            x2, y2 = min(self.obstacles_list_after[i].get_corners(),
                         key=lambda x: (rb.pos[0] - x[0]) ** 2 + (rb.pos[1] - x[1]) ** 2)
            distance = np.sqrt((rb.pos[0] - x1)*(rb.pos[0] - x1) + (rb.pos[1] - y1)*(rb.pos[1] - y1))
            if distance < rb.r:
                distance_next = np.sqrt((rb.pos[0] - x2) * (rb.pos[0] - x2) + (rb.pos[1] - y2) * (rb.pos[1] - y2))
                rb_next = rb.nextPosition(self.goal)
                phi = angle(rb_next[0] - rb.pos[0], rb_next[1] - rb.pos[1], x1 - rb.pos[0], y1 - rb.pos[1])
                phi_next = angle(rb_next[0] - rb.pos[0], rb_next[1] - rb.pos[1], x2 - rb.pos[0], y2 - rb.pos[1])
                decision_temp = fuzzyDecisionMaking(phi, phi_next, distance, distance_next)
                if decision_temp == "Replan":
                    return decision_temp
                elif decision_temp == "Stop":
                    decision = decision_temp
        return decision


def convertphi(phi):
    aS, aD, aF = 0, 0, 0
    if phi > ANGLE_S:
        aS = 1
    elif ANGLE_S >= phi >= ANGLE_DS:
        aS = (ANGLE_S -phi)/(ANGLE_S-ANGLE_DS)
        aD = 1 - (phi-ANGLE_DS)/(ANGLE_S-ANGLE_DS)
    elif ANGLE_DS > phi > ANGLE_DF:
        aD = 1
    elif ANGLE_DF >= phi >= ANGLE_F:
        aD = (ANGLE_DF-phi)/(ANGLE_DF-ANGLE_F)
        aF = 1 - (phi-ANGLE_F)/(ANGLE_DF-ANGLE_F)
    elif ANGLE_F > phi:
        aF = 1
    m = max(aS, aD, aF)
    if m == aS:
        return "S"
    elif m == aD:
        return "D"
    else:
        return "F"


def convertdeltad(deltad):
    ddA, ddU, ddC = 0, 0, 0
    if deltad > DELTAD_A:
        ddA = 1
    elif DELTAD_A >= deltad >= DELTAD_UA:
        ddA = (DELTAD_A - deltad) / (DELTAD_A - DELTAD_UA)
        ddU = (deltad - DELTAD_UA) / (DELTAD_A - DELTAD_UA)
    elif DELTAD_UA > deltad > DELTAD_UC:
        ddU = 1
    elif DELTAD_UC > deltad > DELTAD_C:
        ddU = (DELTAD_UC - deltad) / (DELTAD_UC - DELTAD_C)
        ddC = (deltad - DELTAD_C) / (DELTAD_UC - DELTAD_C)
    elif DELTAD_C > deltad:
        ddC = 1
    m = max(ddA, ddU, ddC)
    if m == ddA:
        return "A"
    elif m == ddU:
        return "U"
    else:
        return "C"


def convertdeltaphi(deltaphi):
    dpA, dpLA, dpU, dpLC, dpC = 0, 0, 0, 0, 0
    if deltaphi > DELTAPHI_A:
        dpA = 1
    elif DELTAPHI_A >= deltaphi >= DELTAPHI_LAA:
        dpA = (DELTAPHI_A - deltaphi) / (DELTAPHI_A - DELTAPHI_LAA)
        dpLA = (deltaphi - DELTAPHI_LAA) / (DELTAPHI_A - DELTAPHI_LAA)
    elif DELTAPHI_LAA > deltaphi > DELTAPHI_LAU:
        dpLA = 1
    elif DELTAPHI_LAU >= deltaphi >= DELTAPHI_ULA:
        dpLA = (DELTAPHI_LAU - deltaphi) / (DELTAPHI_LAU - DELTAPHI_ULA)
        dpU = (deltaphi - DELTAPHI_ULA) / (DELTAPHI_LAU - DELTAPHI_ULA)
    elif DELTAPHI_ULA > deltaphi > DELTAPHI_ULC:
        dpU = 1
    elif DELTAPHI_ULC >= deltaphi >= DELTAPHI_LCU:
        dpU = (DELTAPHI_ULC - deltaphi) / (DELTAPHI_ULC - DELTAPHI_LCU)
        dpLC = (deltaphi -DELTAPHI_LCU) / (DELTAPHI_ULC - DELTAPHI_LCU)
    elif DELTAPHI_LCU > deltaphi > DELTAPHI_LCC:
        dpLC = 1
    elif DELTAPHI_LCC >= deltaphi >= -7:
        dpLC = (DELTAPHI_LCC - deltaphi) / (DELTAPHI_LCC - DELTAPHI_C)
        dpC = (deltaphi - DELTAPHI_C) / (DELTAPHI_LCC - DELTAPHI_C)
    elif -7 > deltaphi:
        dpC = 1
    m = max(dpA, dpLA, dpU, dpLC, dpC)
    if m == dpA:
        return "A"
    elif m == dpLA:
        return "LA"
    elif m == dpU:
        return "U"
    elif m == dpLC:
        return "LC"
    else:
        return "C"


def truthtable(phi, deltad, deltaphi):
    if deltad == "A" :
        return "No"
    elif deltad == "U":
        if phi == "S":
            return "No"
        elif phi == "D":
            if deltaphi == "C" or deltaphi == "LC":
                return "Replan"
            elif deltaphi == "U" or deltaphi == "LA" or deltaphi == "A":
                return "No"
        elif phi == "F":
            if deltaphi == "C" or deltaphi == "LC" or deltaphi == "U":
                return "Stop"
            elif deltaphi == "LA" or deltaphi == "A":
                return "No"
    elif deltad == "C":
        if phi == "S":
            if deltaphi == "LC":
                return "Stop"
            elif deltaphi == "U":
                return "Replan"
            elif deltaphi == "C" or deltaphi == "LA" or deltaphi == "A":
                return "No"
        elif phi == "D":
            if deltaphi == "C" or deltaphi == "LC":
                return "Stop"
            elif deltaphi == "U":
                return "Replan"
            elif deltaphi == "LA" or deltaphi == "A":
                return "No"
        elif phi == "F":
            if deltaphi == "C" or deltaphi == "LC" or deltaphi == "U":
                return "Replan"
            elif deltaphi == "LA" or deltaphi == "A":
                return "No"
    return "No"


def fuzzyDecisionMaking(phit, phit_next, dt, dt_next):
    phi = convertphi(phit / np.pi * 180)
    deltaphi = convertdeltaphi((phit_next - phit) / np.pi * 180)
    deltad = convertdeltad((dt_next - dt))
    return truthtable(phi, deltad, deltaphi)

def angle(x1, y1, x2, y2):
    return np.arccos((x1 * x2 + y1 * y2) / (np.sqrt(x1 * x1 + y1 * y1) * np.sqrt(x2 * x2 + y2 * y2) +1e-6))