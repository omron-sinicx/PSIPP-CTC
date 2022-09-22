#!/usr/bin/env python3

# Copyright (c) 2022 OMRON SINIC X Corporation
# Author: Kazumi Kasaura

import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

import matplotlib.pyplot as plt
plt.rcParams["font.size"] = 16

import matplotlib.patches as patches
import sys
import matplotlib.path
import re

map_names=(
    'den520d',
    'room',
    'random',
    'maze',
    'warehouse',
    'Berlin',
    'empty256',
)
sizes=(
    r'$256\times 256$',
    r'$64\times 64$',
    r'$64\times 64$',
    r'$128\times 128$',
    r'$164\times 340$',
    r'$256\times 256$',
    r'$256\times 256$' 
    )

fig = plt.figure(figsize = (7,4))

for index, map_name in enumerate(map_names):
   with open('../problem_instances/spatial/' + map_name + '.txt', "r") as map_file:
      ax = fig.add_subplot(2, 4, index + 1 if index < 4 else index+2)
   
      n_holes, tasks = map(int, map_file.readline().split())
      n_vertices = int(map_file.readline())
      vertices = []
      
      for i in range(n_vertices):
          x, y = map(float, map_file.readline().split())
          vertices.append([x,y])
      xlim = [min([p[0] for p in vertices]),max([p[0] for p in vertices])]
      ylim = [min([p[1] for p in vertices]),max([p[1] for p in vertices])]
      ax.set_xlim(xlim)
      ax.set_ylim(ylim)
      ax.add_patch(patches.Rectangle([xlim[0],ylim[0]], xlim[1] - xlim[0], ylim[1] - ylim[0], edgecolor = 'k', fill = True, facecolor = 'k', lw = 1))
      ax.add_patch(patches.Polygon(vertices, facecolor = 'w', edgecolor = 'k'))
      for j in range(n_holes):
          n_vertices = int(map_file.readline())
          vertices = []
          for i in range(n_vertices):
              x, y = map(float, map_file.readline().split())
              vertices.append([x,y])
          ax.add_patch(patches.Polygon(vertices, color = 'k'))
   
   
      ax.axis('square')
      ax.axis('off')
      ax.invert_yaxis()
      fig.subplots_adjust(left=0, right=1, bottom=0, top=0.83, hspace = 0.5, wspace = 0.)
      ax.set_xticklabels([])
      ax.set_yticklabels([])
      name = map_name
      if name == 'empty256':
          name = 'empty'
      if name == 'den520d':
          name = 'den'
      ax.set_title(name + '\n' + sizes[index])

plt.savefig('figures/maps.pdf')
plt.show()
   
