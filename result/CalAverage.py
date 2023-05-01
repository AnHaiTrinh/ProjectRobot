import numpy as np
import os

#Change directory if needed
scenario = "room"
allfile = os.listdir("C:/Users/ADMIN/Documents/GitHub/ProjectRobot/result/"+scenario)
print(allfile)
result_average = []
result_variance = []

for file in allfile:
    with open("C:/Users/ADMIN/Documents/GitHub/ProjectRobot/result/"+scenario+"/"+file) as f:
        result_temp = []
        flag = False

        line = f.readline()
        tempstr = []
        while line.startswith(scenario[0]):
            flag = True
            mystr = line.split()
            for arr in mystr:
                if arr.replace('.', '', 1).isdigit():
                    tempstr.append(float(arr))
            result_temp.append(tempstr)
            line = f.readline()
            tempstr = []

        result_average.append(np.mean(np.array(result_temp), axis=0, dtype=float))
        result_variance.append(np.std(np.array(result_temp), axis=0, dtype=float))

print(f"Astar: {result_average[0]} {result_variance[0]}")
print(f"Grid: {result_average[1]} {result_variance[1]}")
print(f"OnlyReplan: {result_average[2]} {result_variance[2]}")
print(f"Quad_Dstar_Fuzzy: {result_average[3]} {result_variance[3]}")