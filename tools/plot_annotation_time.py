#!/usr/bin/env python3

# Copyright (c) 2022 OMRON SINIC X Corporation
# Author: Kazumi Kasaura

import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

import matplotlib.pyplot as plt
plt.rcParams["font.size"] = 22

fig, ax = plt.subplots(figsize = (7,5))

labels=['100 PRM',
        '100 CDT',
        '700 PRM',
        '700 CDT',
        '5000 PRM',
        '5000 CDT',
        'maze',
        'room',
        'random',
        'warehouse',
        'empty',
        'Berlin',
        'Result 3',
]
runtimes = [69.72, 62.64, 883.32, 129.32, 10360.32, 872.92,
            85.76, 96.12, 110.36, 686.28, 70.32, 164.88, 9949.12]
ax.barh(range(13), width = runtimes, tick_label = labels)
fig.subplots_adjust(left=0.26, right=0.99, bottom=0.15, top=1.00)
plt.xscale("log")
ax.set_xlabel("Time (ms)")
ax.invert_yaxis()
plt.savefig("./experiment_plots/annotation.pdf")
plt.show()
