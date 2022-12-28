import numpy as np

ANGLE_F = 25
ANGLE_DF = 35
ANGLE_DS = 55
ANGLE_S = 65

DELTAD_A = 1.5
DELTAD_UA = 1
DELTAD_UC = -1
DELTAD_C = -1.5

DELTAPHI_A = 7
DELTAPHI_LAA = 6
DELTAPHI_LAU = 4
DELTAPHI_ULA = 3
DELTAPHI_ULC = -3
DELTAPHI_LCU = -4
DELTAPHI_LCC = -6
DELTAPHI_C = -7


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