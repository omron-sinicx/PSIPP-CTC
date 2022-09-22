#!/usr/bin/env python3

# Copyright (c) 2022 OMRON SINIC X Corporation
# Author: Kazumi Kasaura


import matplotlib.pyplot as plt 
import matplotlib.patches as patches
import sys
import matplotlib.animation as animation
import math
import numpy as np
import matplotlib.path

time_span = float(sys.argv[3])
    
fig, ax = plt.subplots(figsize = (7,7))
if sys.argv[1] != "-":
    with open(sys.argv[1], "r") as map_file:
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

if len(sys.argv) > 5:
    with open(sys.argv[5], "r") as roadmap_file:
        n, m, k = map(int, roadmap_file.readline().split())
        
        points = []
        for i in range(n):
            point = list(map(float, roadmap_file.readline().split()))
            points.append(point)
  
        plt.scatter([p[0] for p in points], [p[1] for p in points], s = 1.0)
  
        for i in range(m):
            u, v = list(map(int, roadmap_file.readline().split()))
            path = matplotlib.path.Path([points[u], points[v]])
            ax.add_patch(patches.PathPatch(path, linewidth = 0.1, fill = False))
  
        tasks = []
  
        for i in range(k):
            tasks.append(list(map(int, roadmap_file.readline().split())))
  
        agent_size = float(roadmap_file.readline())
     
        colors = plt.cm.get_cmap('hsv', k)
        for i in range(k):
            s,t = tasks[i]

        xlim = [min([point[0] for point in points]), max([point[0] for point in points])]
        ylim = [min([point[1] for point in points]), max([point[1] for point in points])]
            
    ax.invert_yaxis()
    ax.axis('off')
    fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    r = 0.3
    fig.set_figwidth(r * (xlim[1]-xlim[0]))
    fig.set_figheight(r * (ylim[1]-ylim[0]))
        
with open(sys.argv[2], "r") as plan_file:
    plan=[]
    line = plan_file.readline().split()
    number_of_agents = int(line[0])
    agent_size = float(line[1])

    makespan = 0.0
    routes = []
    
    for i in range(number_of_agents):
        route = []
        route_length = int(plan_file.readline())
        for j in range(route_length):
            time = float(plan_file.readline())
            point = list(map(float, plan_file.readline().split()))
            route.append([time, point])

        makespan = max(makespan, route[len(route)-1][0])
        routes.append(route)

    steps = number_of_agents * [0]
    agent_circles = []

    colors = plt.cm.get_cmap('jet', number_of_agents)
    
    def init():
        global steps
        global routes
            
        steps = number_of_agents * [0]
        return []
    
    agent_circles = []
    
    def plot(frame):
        global steps
        global routes
        global agent_circles
        for circle in agent_circles:
            circle.remove()

        t = time_span * frame

        agent_circles = []
        for agent in range(number_of_agents):
            while steps[agent] < len(routes[agent]) and routes[agent][steps[agent]][0] < t:
                steps[agent]+=1
            step = steps[agent]
            if step == len(routes[agent]):
                position = routes[agent][step-1][1]
            else:
                r = (t - routes[agent][step-1][0]) / (routes[agent][step][0] - routes[agent][step-1][0])
                position = (1-r) * np.array(routes[agent][step-1][1]) + r * np.array(routes[agent][step][1])

            agent_circles.append(ax.add_patch(patches.Circle(position, radius = agent_size, color = colors(agent))))
        return agent_circles
    
    ani = animation.FuncAnimation(fig, plot, frames = range(math.ceil(makespan / time_span)), interval=100, init_func = init, blit = True)
    plt.show()
    if len(sys.argv) > 4:
        #ani.save(sys.argv[4], writer='ffmpeg')
        ani.save(sys.argv[4], writer='imagemagick')
