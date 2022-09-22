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
    'room_700_CDT.txt',
    'random_700_CDT.txt',
    'maze_700_CDT.txt',
    'warehouse_700_CDT.txt',
    'Berlin_700_CDT.txt',
    'empty256_700_CDT.txt',
)

fig = plt.figure(figsize = (7,4))

indices = (1,4,2,5,3,6)

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
    
    results = {"Annotated-PSIPP":[[],[]], "Unannotated-PSIPP":[[],[]], "CCBS":[[],[]],}
    colors = ('tab:blue', 'tab:cyan', 'tab:orange')
    labels = ('PSIPP/CTC', 'PSIPP', 'CCBS')

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
    ax = fig.add_subplot(2, 3, indices[index])
    name = exp_name.split('_')[0]
    name=re.sub(r'[0-9]+', '', name)
    ax.set_title(name)

    max_n_agent = 0
    for index, name in enumerate(results):
        result = results[name]
        max_n_agent = max(max_n_agent, len(result))
        if len(result[len(result)-1]) == 0:
            result = result[:len(result)-1]
        n_agents = []
        times = []
        for n_agent, res in enumerate(result):
            for tpl in res:
                n_agents.append(n_agent)
                times.append(tpl[3])
        ax.scatter(n_agents, times, label = labels[index], color = colors[index], s = 1, rasterized = True)
    
    plt.xlim([1, 701])
    plt.ylim([1, 30000])
    plt.yscale("log")

plt.legend(bbox_to_anchor=(-0.8, -0.6), loc="lower center", ncol = 3,markerscale=12)
fig.subplots_adjust(left=0.09, right=0.97, bottom=0.18, top=0.92, hspace = 0.4, wspace = 0.3)
plt.savefig("./experiment_plots/runtime_2.pdf", dpi = 400)
plt.show()
