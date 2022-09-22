#!/usr/bin/env python3

# Copyright (c) 2022 OMRON SINIC X Corporation
# Author: Kazumi Kasaura

import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

import matplotlib.pyplot as plt
plt.rcParams["font.size"] = 12

fig, ax = plt.subplots(figsize=(2.5,2.5))

#ax.set_ylabel("Percentile")
#ax.set_xlabel("SOC ratio (PSIPP/CCBS)")

ratios = {}
max_ratio = 0.0

with open("./experiment_plots/soc_ratio.txt", "r") as in_file:
    while True:
        try:
            name = in_file.readline()
        except EOFError:
            break
        if name == "":
            break
        n_cases = int(in_file.readline())
        ratios[name] = []
        for i in range(n_cases):
            ratio = float(in_file.readline())
            max_ratio = max(max_ratio, ratio)
            ratios[name].append(ratio)

max_ratio = 1.3553986540883678

for name, ratio in ratios.items():
    if name[0] == 'd':
        N = name.split('_')[1]
        gen = name.split('_')[2]
        if gen[0] == 'k':
            gen = gen[1:]
        ax.plot(ratio+[max_ratio], [100 * i / len(ratio) for i in range(1, len(ratio)+1)]+[100], label = N+', ' + gen[:-1])
    count1 = sum([1 if r<1.001 else 0 for r in ratio])
    count10 = sum([1 if r<1.1 else 0 for r in ratio])
    print(name, count1/len(ratio), ' ', count10/len(ratio))
ax.legend(bbox_to_anchor=(1.0, 0.45), loc="center right")
fig.subplots_adjust(left=0.16, right=0.99, bottom=0.1, top=0.99)
plt.savefig("./experiment_plots/soc_ratio.pdf")
plt.show()
