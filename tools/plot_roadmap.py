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

fig, ax = plt.subplots()
if sys.argv[1] != "-":
   with open(sys.argv[1], "r") as map_file:
   
      ex = sys.argv[1][len(sys.argv[1])-3:]
      if ex == "map":
   
         map_file.readline()
         height = int(map_file.readline().split()[1])
         width = int(map_file.readline().split()[1])
         map_file.readline()
         for y in range(height):
            line = map_file.readline()
            for x in range(width):
               c = line[x]
               if c != '.' and c != 'G' and c != 'S':
                  ax.add_patch(patches.Rectangle((x, y), 1., 1., facecolor = 'black'))
         plt.xlim([0, width])
         plt.ylim([0, height])
   
      elif ex == "txt":
         n_holes, tasks = map(int, map_file.readline().split())
         n_vertices = int(map_file.readline())
         vertices = []
         
         for i in range(n_vertices):
            x, y = map(float, map_file.readline().split())
            vertices.append([x,y])
         xlim = [min([p[0] for p in vertices]),max([p[0] for p in vertices])]
         ylim = [min([p[1] for p in vertices]),max([p[1] for p in vertices])]
         plt.xlim(xlim[0], xlim[1])
         plt.ylim(ylim[0], ylim[1])
         ax.add_patch(patches.Rectangle([xlim[0],ylim[0]],xlim[1]-xlim[0],ylim[1]-ylim[0], color = 'k'))
         ax.add_patch(patches.Polygon(vertices, facecolor = 'w', edgecolor = 'k'))
         for j in range(n_holes):
            n_vertices = int(map_file.readline())
            vertices = []
            for i in range(n_vertices):
               x, y = map(float, map_file.readline().split())
               vertices.append([x,y])
            ax.add_patch(patches.Polygon(vertices, color = 'k'))
   
            
if len(sys.argv) > 2:
   with open(sys.argv[2], "r") as roadmap_file:
      n, m, k = map(int, roadmap_file.readline().split())
      
      points = []
      for i in range(n):
         point = list(map(float, roadmap_file.readline().split()))
         points.append(point)

      plt.scatter([p[0] for p in points], [p[1] for p in points], s = 1.0)
      xlim = [min([p[0] for p in points]),max([p[0] for p in points])]
      ylim = [min([p[1] for p in points]),max([p[1] for p in points])]

      for i in range(m):
         u, v = list(map(int, roadmap_file.readline().split()))
         path = matplotlib.path.Path([points[u], points[v]])
         ax.add_patch(patches.PathPatch(path, linewidth = 0.1, fill = False))

      tasks = []


      for i in range(k):
         tasks.append(list(map(int, roadmap_file.readline().split())))

      agent_size = float(roadmap_file.readline())
   
      colors = plt.cm.get_cmap('jet', k)
      for i in range(k):
         s,t = tasks[i]
         ax.add_patch(patches.Circle(points[s], radius=agent_size, color = colors(i)))
         ax.add_patch(patches.Circle(points[t], fill = False, radius=agent_size, color = colors(i)))

ax.invert_yaxis()
ax.axis('off')
fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
ax.set_xticklabels([])
ax.set_yticklabels([])
r = 0.2
fig.set_figwidth(r * (xlim[1]-xlim[0]))
fig.set_figheight(r * (ylim[1]-ylim[0]))
plt.savefig('figures/'+sys.argv[1].split('/')[-1].split('.')[0]+'.png')
plt.show()
