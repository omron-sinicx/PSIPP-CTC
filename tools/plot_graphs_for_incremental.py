#!/usr/bin/env python3

# Copyright (c) 2022 OMRON SINIC X Corporation
# Author: Kazumi Kasaura

import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

import matplotlib.pyplot as plt
plt.rcParams["font.size"] = 14

import sys
n_all_agent = int(sys.argv[1])
iterations = int(sys.argv[2])
n_perm = 1
in_file = open(sys.argv[3], "r")
exp_name = sys.argv[3].split('/')[-1].split('.')[0]

annotation=0.
for it in range(iterations):
    line=in_file.readline().split();
    annotation+=int(line[2])
annotation/=iterations
print("annotation: ",annotation)

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

fig, ax = plt.subplots(figsize = (3.5, 2.5))

# ax.set_xlabel("number of agents")
# ax.set_ylabel("success rate")

max_n_agent = 0

for index, name in enumerate(results):
    result = results[name]
    ax.plot(range(1, len(result)), [len(res)/iterations for res in result[1:]], label = labels[index], color = colors[index], lw = lws[index])
    max_n_agent = max(max_n_agent, len(result))

plt.xlim([1, max_n_agent])
plt.ylim([0.0,1.0])
ax.legend()
fig.subplots_adjust(left=0.17, right=0.99, bottom=0.12, top=0.95)
plt.savefig("./experiment_plots/"+exp_name+"_success_rate.png")
plt.show()

fig, ax = plt.subplots(figsize = (3.5, 2.5))

# ax.set_xlabel("number of agents")
# ax.set_ylabel("time (ms)")

for index, name in enumerate(results):
    result = results[name]
    if len(result[len(result)-1]) == 0:
        result = result[:len(result)-1]
    n_agents = []
    times = []
    for n_agent, res in enumerate(result):
        for tpl in res:
            n_agents.append(n_agent)
            times.append(tpl[3])
    ax.scatter(n_agents, times, label = labels[index], color=colors[index], s = 1, rasterized = True)

plt.xlim([1, max_n_agent])

ax.legend(markerscale=12)
fig.subplots_adjust(left=0.21, right=0.99, bottom=0.12, top=1.00)
plt.savefig("./experiment_plots/"+exp_name+"_time.png", dpi = 400)
plt.show()

fig, ax = plt.subplots(figsize = (3.5, 2.5))

ax.set_ylabel("Percentile")
ax.set_xlabel("SOC ratio (PSIPP / CCBS)")

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
        if PSIPP_success[subtask] and CCBS_success[subtask]:
            rate = PSIPP_success[subtask][1] / CCBS_success[subtask][1]
            soc_rates.append(rate)

soc_rates.sort()
n_cases = len(soc_rates)
ax.plot(soc_rates, [ 100 * i / n_cases for i in range(1, n_cases+1)])

plt.savefig("./experiment_plots/"+exp_name+"_soc_ratio.png")
plt.show()
