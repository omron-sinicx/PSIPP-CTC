#!/usr/bin/env python3

# Copyright (c) 2022 OMRON SINIC X Corporation
# Author: Kazumi Kasaura

import pybullet as p
import time
import math

import numpy
import copy
import quaternion
import sys
from PIL import Image

p.connect(p.GUI)

plane = p.loadURDF("../tools/simulation/plane.urdf")
#p.setRealTimeSimulation(1)
#p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
#p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

mass = 0.6
inertia = 0.0375 / 0.125 / 0.125 / 2
wheel_position = 0.125
wheel_radius = 0.05
time_ratio = 0.5
maxVelocity = time_ratio/wheel_radius * 1.01
acceralation = maxVelocity * 0.1 /2.
maxForce = 1000.

offset_time = 0.10

p.setGravity(0, 0, -10)
time.sleep(1./240.)        
    
with open(sys.argv[1], "r") as plan_file:
    line = plan_file.readline().split()
    number_of_agents = int(line[0])
    agent_size = float(line[1])

    makespan = 0.0
    routes = []
    
    for i in range(number_of_agents):
        route = []
        route_length = int(plan_file.readline())
        for j in range(route_length):
            arrival = float(plan_file.readline())
            point = list(map(float, plan_file.readline().split()))
            route.append([arrival, point])

        makespan = max(makespan, route[len(route)-1][0])
        routes.append(route)

toios=[]
for i in range(number_of_agents):
    offset = routes[i][0][1] + [0.]
    toios.append(p.loadURDF("../tools/simulation/toio.urdf", offset))

currents=number_of_agents * [0]

initial_time = time.time()

living = True
current_time = 0.
#p.startStateLogging(
#            p.STATE_LOGGING_VIDEO_MP4,
#    "video.mp4")

#while True:
#    print(p.getDebugVisualizerCamera())
#    dist, yaw, pitch, x, y, z = map(float, sys.stdin.readline().split())
#    p.resetDebugVisualizerCamera(dist,yaw,pitch,[x,y,z])

#sys.stdin.read()
if number_of_agents <= 15:
    dist = 3
    yaw = 90
    pitch = -55
    target = [3., 0., 0.]
    p.resetDebugVisualizerCamera(dist, yaw, pitch, target)
    c_matrix  = p.computeViewMatrixFromYawPitchRoll(target, dist, yaw, pitch,0.,1)
    video_speed = 2
elif number_of_agents <= 70:
    dist = 9
    yaw = 90
    pitch = -55
    target = [9., 0., 0.]
    p.resetDebugVisualizerCamera(dist, yaw, pitch, target)
    c_matrix  = p.computeViewMatrixFromYawPitchRoll(target, dist, yaw, pitch,0.,1)
    video_speed = 4
else:
    dist = 14.5
    yaw = 90
    pitch = -55
    target = [18., 0., 0.]
    p.resetDebugVisualizerCamera(dist, yaw, pitch, target)
    c_matrix  = p.computeViewMatrixFromYawPitchRoll(target, dist, yaw, pitch,0.,1)
    video_speed = 8

time.sleep(1./240.)
#print(c_height, c_width, c_matrix, p_matrix)
c_height = 1200
c_width = 1008

images=[]
step=0
skip = 10 * video_speed * 2

while living:
    p.stepSimulation()
    time.sleep(1./240.)
    step+=1
    current_time+=1./240. * time_ratio
    if step % 24 == 0:
        print(current_time)
    if step % skip == 0:
        camera_returned = p.getCameraImage(c_height, c_width, c_matrix)
        #image = Image.fromarray(numpy.asarray(camera_returned[2]).astype('uint8'))
        image = numpy.asarray(camera_returned[2]).astype('uint8')
        if step == skip:
            pngimage = Image.fromarray(image)
            pngimage.save('simulation/1.png')
        images.append(image)
    contact_points = p.getContactPoints()
    collision = False
    for contact_point in contact_points:
        if contact_point[1] != plane and contact_point[2] != plane:
            #current_time = (time.time() - initial_time) * time_ratio
            i = contact_point[1] - 1
            j = contact_point[2] - 1
            position_i = p.getBasePositionAndOrientation(toios[i])[0]
            position_j = p.getBasePositionAndOrientation(toios[j])[0]
            print(position_i, position_j)
            print(current_time)
            print(routes[i][currents[i]], routes[i][currents[i]+1])
            print(routes[j][currents[j]], routes[j][currents[j]+1])
            collision = True
            break
    if collision:
        print("collision!")
        break
    goaled = 0
    for i in range(number_of_agents):
        if currents[i] == len(routes[i])-1:
            p.setJointMotorControl2(toios[i], 0, p.VELOCITY_CONTROL,
                                    targetVelocity=0, force=maxForce)
            p.setJointMotorControl2(toios[i], 1, p.VELOCITY_CONTROL,
                                    targetVelocity=0, force=maxForce)
            goaled+=1
            continue

        state = p.getBasePositionAndOrientation(toios[i])
        position = state[0][:2]
        quat = numpy.quaternion(state[1][3], state[1][0], state[1][1], state[1][2])
        direction = quaternion.rotate_vectors(quat, (1.,0.,0.))[:2]
        target = numpy.array(routes[i][currents[i]+1][1])
        if numpy.linalg.norm(target-position) < 0.01:
            target_time = routes[i][currents[i]+1][0]
            #current_time = (time.time() - initial_time) * time_ratio
            print(i, current_time, target_time)
            while currents[i]+1<len(routes[i]) and  numpy.linalg.norm(numpy.array(routes[i][currents[i]+1][1])-position) < 0.01:
                currents[i]+=1
            continue
        theta_diff = math.atan2(target[1]-position[1], target[0]-position[0]) - math.atan2(direction[1], direction[0])
        if theta_diff > numpy.pi:
            theta_diff -= 2 * numpy.pi
        if theta_diff < -numpy.pi:
            theta_diff += 2 * numpy.pi

        #if i==38:
        #    start_time = routes[i][currents[i]][0] - offset_time
        #    target_time = routes[i][currents[i]+1][0]
            #current_time = (time.time() - initial_time) * time_ratio
        #    print(current_time, theta_diff, start_time, target_time)
        #    print(position, target)

        if theta_diff > 0.1:
            p.setJointMotorControl2(toios[i], 0, p.VELOCITY_CONTROL,
                                    targetVelocity=maxVelocity, force=maxForce)
            p.setJointMotorControl2(toios[i], 1, p.VELOCITY_CONTROL,
                                    targetVelocity=-maxVelocity, force=maxForce)
        elif theta_diff < -0.1:
            p.setJointMotorControl2(toios[i], 0, p.VELOCITY_CONTROL,
                                    targetVelocity=-maxVelocity, force=maxForce)
            p.setJointMotorControl2(toios[i], 1, p.VELOCITY_CONTROL,
                                    targetVelocity=maxVelocity, force=maxForce)
        else:
            start_time = routes[i][currents[i]][0] - offset_time
            target_time = routes[i][currents[i]+1][0]
            #current_time = (time.time() - initial_time) * time_ratio
            if current_time < start_time:
                if theta_diff > 0.01:
                    p.setJointMotorControl2(toios[i], 0, p.VELOCITY_CONTROL,
                                            targetVelocity=0.1*maxVelocity, force=maxForce)
                    p.setJointMotorControl2(toios[i], 1, p.VELOCITY_CONTROL,
                                            targetVelocity=-0.1*maxVelocity, force=maxForce)
                elif theta_diff < -0.01:
                    p.setJointMotorControl2(toios[i], 0, p.VELOCITY_CONTROL,
                                            targetVelocity=-0.1*maxVelocity, force=maxForce)
                    p.setJointMotorControl2(toios[i], 1, p.VELOCITY_CONTROL,
                                            targetVelocity=0.1*maxVelocity, force=maxForce)
                else:
                    p.setJointMotorControl2(toios[i], 0, p.VELOCITY_CONTROL,
                                            targetVelocity=0, force=maxForce)
                    p.setJointMotorControl2(toios[i], 1, p.VELOCITY_CONTROL,
                                            targetVelocity=0, force=maxForce)
            elif current_time > target_time + 0.1:
                print(i)
                print(routes[i][currents[i]],routes[i][currents[i]+1])

                print("too late!")
                living = False
                break
            else:
                velocity=maxVelocity
                if target_time-current_time > numpy.linalg.norm(target-position):
                    velocity*=0.9
                if theta_diff > 0.01:
                    p.setJointMotorControl2(toios[i], 0, p.VELOCITY_CONTROL,
                                            targetVelocity=1.1*velocity, force=maxForce)
                    p.setJointMotorControl2(toios[i], 1, p.VELOCITY_CONTROL,
                                            targetVelocity=0.9*velocity, force=maxForce)
                elif theta_diff < -0.01:
                    p.setJointMotorControl2(toios[i], 0, p.VELOCITY_CONTROL,
                                            targetVelocity=0.9*velocity, force=maxForce)
                    p.setJointMotorControl2(toios[i], 1, p.VELOCITY_CONTROL,
                                            targetVelocity=1.1*velocity, force=maxForce)
                else:
                    p.setJointMotorControl2(toios[i], 0, p.VELOCITY_CONTROL,
                                            targetVelocity=velocity, force=maxForce)
                    p.setJointMotorControl2(toios[i], 1, p.VELOCITY_CONTROL,
                                            targetVelocity=velocity, force=maxForce)
    if goaled == number_of_agents:
        print('finished')
        break
        

import imageio.v2 as imageio

writer = imageio.get_writer('simulation/video.mp4',fps=24)
for image in images:
    writer.append_data(image)
writer.close()
