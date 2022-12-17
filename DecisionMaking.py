import math


def convertphi(phi):
    aS, aD, aF = 0, 0, 0
    if phi > 65:
        aS = 1
    elif 65 >= phi >= 55:
        aS = (phi-55)/10
        aD = 1 - (phi-55)/10
    elif 55 > phi > 35:
        aD = 1
    elif 35 >= phi >= 25:
        aD = (phi-25)/10
        aF = 1 - (phi-25)/10
    elif 25 > phi:
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
    if deltad > 3:
        ddA = 1
    elif 3 >= deltad >= 2:
        ddA = deltad - 2
        ddU = 3 - deltad
    elif 2 > deltad >-2:
        ddU = 1
    elif -2 > deltad > -3:
        ddU = deltad + 3
        ddC = -2 - deltad
    elif -3 > deltad:
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
    if deltaphi > 7:
        dpA = 1
    elif 7 >= deltaphi >=6:
        dpA = deltaphi - 6
        dpLA = 7 - deltaphi
    elif 6 > deltaphi > 4:
        dpLA = 1
    elif 4 >= deltaphi >= 3:
        dpLA = deltaphi - 3
        dpU =  4 - deltaphi
    elif 3 > deltaphi > -3:
        dpU = 1
    elif -3 >= deltaphi >= -4:
        dpU = deltaphi + 3
        dpLC = -2 - deltaphi
    elif -4 > deltaphi > -6:
        dpLC = 1
    elif -6 >= deltaphi >= -7:
        dpLC = deltaphi + 7
        dpC = -6 - deltaphi
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
    phi = convertphi(phit / math.pi * 180)
    deltaphi = convertdeltaphi((phit_next - phit) / math.pi * 180)
    deltad = convertdeltad((dt_next - dt)/5)
    return truthtable(phi, deltad, deltaphi)