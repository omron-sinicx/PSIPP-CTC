#!/usr/bin/env python3

# Copyright (c) 2022 OMRON SINIC X Corporation
# Author: Kazumi Kasaura

import matplotlib.pyplot as plt
names = []
gptime = []
gctime = []
emtime = []
eptime = []
while True:
    try:
        line = input().split(",")
    except EOFError:
        break
    names.append(line[0])
    gptime.append(float(line[1]) - float(line[2]))
    gctime.append(float(line[2]))
    emtime.append(float(line[3]))
    eptime.append(float(line[4]))


xpos = list(range(len(names)))
fig, ax = plt.subplots()
width = 0.3
ax.bar([x - width/2 for x in xpos], gptime, label="planning w/o CC", width = width)
ax.bar([x - width/2 for x in xpos], gctime, bottom=gptime, label="CC in time", width = width)
ax.bar([x + width/2 for x in xpos], eptime, label="planning", width = width)
ax.bar([x + width/2 for x in xpos], emtime, bottom=eptime, label="precalc", width = width)

ax.set(xticks=xpos, xticklabels=names)

ax.set_ylabel("time(ms)")
ax.legend()

plt.show()
