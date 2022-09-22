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

import matplotlib.colors as mcolors
import re

exp_names=(
    'den520d_100_kPRM.txt',
    'den520d_100_CDT.txt',
    'den520d_700_kPRM.txt',
    'den520d_700_CDT.txt',
    'den520d_5000_kPRM.txt',
    'den520d_5000_CDT.txt',
    'room_700_CDT.txt',
    'random_700_CDT.txt',
    'maze_700_CDT.txt',
    'warehouse_700_CDT.txt',
    'Berlin_700_CDT.txt',
    'empty256_700_CDT.txt',
)

fig = plt.figure(figsize = (7, 8))

indices = (1,4,2,5,3,6,7,10,8,11,9,12)

for index, exp_name in enumerate(exp_names):
    in_file = open( '../experiments/results/' + exp_name, "r")
    n_all_agent = int(exp_name.split('_')[1])
    
    annotation=0.
    for it in range(iterations):
        line=in_file.readline().split();
        annotation+=int(line[2])
    annotation/=iterations
    
    results = {"Annotated-PSIPP":[[],[]], "Unannotated-PSIPP":[[],[]], "CCBS":[[],[]],}
    colors = ('tab:blue', 'tab:cyan', 'tab:orange')
    labels = ('PSIPP/CTC', 'PSIPP', 'CCBS')
    lws = (4, 2, 2)
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

    ax = fig.add_subplot(4, 3, indices[index])

    if index < 6:
        ax.set_title('N = '+str(n_all_agent) +', '+ ('PRM' if exp_name.split('_')[2]=='kPRM.txt' else 'CDT'))
    else:
        gname = exp_name.split('_')[0]
        gname=re.sub(r'[0-9]+', '', gname)
        ax.set_title(gname)
    
    max_n_agent = 0

    for index, name in enumerate(results):
        result = results[name]
        ax.plot(range(1, len(result)), [len(res)/iterations for res in result[1:]], label = labels[index], lw = lws[index], color=colors[index])
        max_n_agent = max(max_n_agent, len(result))
    
    xlen = (100 if n_all_agent == 100 else 700)
    asp = xlen * 0.8
    #ax.set_aspect(asp)
    plt.xlim([1, xlen+1])
    plt.ylim([0, 1.0])
    

plt.legend(bbox_to_anchor=(-0.8, -0.55), loc="lower center", ncol = 3)
fig.subplots_adjust(left=0.09, right=0.97, bottom=0.09, top=0.96, hspace = 0.4, wspace = 0.3)
plt.savefig("./experiment_plots/success_rate.pdf")
plt.show()
