#!/usr/bin/env python3

# Copyright (c) 2022 OMRON SINIC X Corporation
# Author: Kazumi Kasaura

import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

import matplotlib.pyplot as plt
plt.rcParams["font.size"] = 12

import sys
iterations = 25
n_perm = 1
import re

exp_names=(
    'maze_700_CDT.txt',
    'room_700_CDT.txt',
    'random_700_CDT.txt',
    'warehouse_700_CDT.txt',
    'empty256_700_CDT.txt',
    'Berlin_700_CDT.txt',
)

fig, ax = plt.subplots(figsize=(2.5,2.5))

#ax.set_ylabel("Percentile")
#ax.set_xlabel("SOC ratio (PSIPP/CCBS)")

ratios = {}
max_ratio = 0.0

for index, exp_name in enumerate(exp_names):
    in_file = open( '../experiments/results/' + exp_name, "r")
    n_all_agent = int(exp_name.split('_')[1])
    
    annotation=0.
    for it in range(iterations):
        line=in_file.readline().split();
        annotation+=int(line[2])
    annotation/=iterations
    with open("./experiment_plots/annotation_time_2.txt", "a") as out:
        out.write(exp_name+'\n')
        out.write(str(annotation)+'\n')
    
    results = {"CCBS":[[],[]], "Unannotated-PSIPP":[[],[]], "Annotated-PSIPP":[[],[]]}
    
    while True:
        try:
            line = in_file.readline().split(",")
        except EOFError:
            break
        if line == [""]:
            break
        if line[1].lstrip() == "CCBS":
            name = "CCBS"
        else:
            name = line[0].lstrip() + '-' + line[1].lstrip()
        n_agent = int(line[2])
        if float(line[5]) < 0.0:
            continue
        results[name][n_agent].append([int(line[3]) * iterations + int(line[4]), float(line[5]), float(line[6]), float(line[7])])
        if(len(results[name]) <= min(n_agent+1, n_all_agent)):
            results[name].append([])
    
    in_file.close()
    CCBS_result = results["CCBS"]
    PSIPP_result = results["Annotated-PSIPP"]

    ns_agent = []

    soc_rates = []

    for n_agent in range(1, min(len(CCBS_result), len(PSIPP_result))):
        PSIPP_success = iterations * n_perm * [False]
        CCBS_success = iterations * n_perm * [False]
        for res in CCBS_result[n_agent]:
            CCBS_success[res[0]] = [res[1], res[2]]
        for res in PSIPP_result[n_agent]:
            PSIPP_success[res[0]] = [res[1], res[2]]
        for subtask in range(0, iterations * n_perm):
            if CCBS_success[subtask]:
                rate = PSIPP_success[subtask][1] / CCBS_success[subtask][1]
                soc_rates.append(rate)


    name = exp_name.split('_')[0]
    name=re.sub(r'[0-9]+', '', name)
    count1 = sum([1 if r<1.01 else 0 for r in soc_rates])
    count10 = sum([1 if r<1.1 else 0 for r in soc_rates])
    print(name, count1/len(soc_rates), ' ', count10/len(soc_rates))
    soc_rates.sort()
    ratios[name] = soc_rates
    max_ratio = max(max_ratio, soc_rates[-1])

for name, soc_rates in ratios.items():
    n_cases = len(soc_rates)
    ax.plot(soc_rates+[max_ratio], [ 100 * i / n_cases for i in range(1, n_cases+1)]+[100], label = name)
    

ax.legend(bbox_to_anchor=(1.0, 0.425), loc="center right")
fig.subplots_adjust(left=0.16, right=0.99, bottom=0.1, top=0.99)
plt.savefig("./experiment_plots/soc_ratio_2.pdf")
print(max_ratio)
plt.show()
