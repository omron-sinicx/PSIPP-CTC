#!/usr/bin/env python3

# Copyright (c) 2022 OMRON SINIC X Corporation
# Author: Kazumi Kasaura

import matplotlib.pyplot as plt
n_maps, n_instaces, iteration = map(int, input().split(','))
input()
results = []
best_socs = n_instaces * [float('inf')]
for t in range(n_maps):
    line = input().split(',')
    name = line[0]
    generation_time = float(line[1])
    runs = []
    for c in range(n_instaces):
        for it in range(iteration):
            line = input().split(',')

            time = float(line[4])
            makespan = float(line[5])
            soc = float(line[6])
            if time >-0.5 and best_socs[c] > soc:
                best_socs[c] = soc
            runs.append([time, makespan, soc, c])
    results.append([name, generation_time, runs])

PRM_result = []
kPRM_result = []
Grid_result = []
CDT_result = []
AASIPP_result = []

for result in results:
    success = 0
    average_time = 0.0
    score = 0.0
    for run in result[2]:
        if run[0] > -0.5:
            success +=1
            average_time += run[0]
            score += best_socs[run[3]] / run[2]
    if success == 0:
        continue
    average_time /= success
    generator = list(result[0].split('-'))[0]
    if generator == 'PRM':
        PRM_result.append([average_time, score])
    elif generator == 'kPRM':
        kPRM_result.append([average_time, score])
    elif generator == 'GRID':
        Grid_result.append([average_time, score])
    elif generator == 'CDT':
        CDT_result.append([average_time, score])
    elif generator == 'AASIPP':
        AASIPP_result.append([average_time, score])

fig, ax = plt.subplots()
        
ax.set_xlabel("time (ms)")
plt.xscale("log")
ax.set_ylabel("Score")
ax.scatter([res[0] for res in Grid_result], [res[1] for res in Grid_result], label = "Grid")
ax.scatter([res[0] for res in PRM_result], [res[1] for res in PRM_result], label = "PRM")
ax.scatter([res[0] for res in kPRM_result], [res[1] for res in kPRM_result], label = "kPRM")
ax.scatter([res[0] for res in CDT_result], [res[1] for res in CDT_result], label = "CDT")
ax.scatter([res[0] for res in AASIPP_result], [res[1] for res in AASIPP_result], label = "AASIPP")

ax.legend()
plt.show()
