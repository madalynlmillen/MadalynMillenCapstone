import sys
import numpy as np
from PIL import Image
from cv2 import *
import BlobDetection
from copy import deepcopy
from mpl_toolkits.mplot3d import Axes3D
import ConfigSpaceConversion
import matplotlib.pyplot as plt
from RobotWorld import *
import os.path
import PathPlanning
import hebi
from time import sleep, time
import math
import random
'''
#Code inspired by https://www.pyimagesearch.com/2016/02/08/opencv-shape-detection/
'''

'''
This function takes the list of obstacle points collected by BlobDetection and
creates a list of boolean values. This list will assign a spot as false if there is an obstacle
there, and be true otherwise. This list will then be passed to the path planning function.
'''
def convertObjects():
    args, image = BlobDetection.takePhoto()
    listOfObjects, obstacleInfo, boxPointsList = BlobDetection.detectAndDraw(args, image)
    imageList = np.array(image, dtype='float')

    width = len(imageList)
    height = len(imageList[0])

    imageBoolList = np.ones((width, height), dtype=bool)
    newSpaceImage = np.zeros((width, height, 3), dtype=np.uint8)
    #This loop assigns a false value to spots with obstacles in them and helps create an image that tells where the obstacles are
    for j in listOfObjects:
        for pair in j:
            a,b = pair
            if b < width and a < height:
                imageBoolList[int(b)][int(a)] = False
                newSpaceImage[int(b)][int(a)] = [255, 255, 255]

    #This image is created from the boolean list to show where the obstacles are
    newSpaceImage = Image.fromarray(newSpaceImage, 'RGB')
    newSpaceImage.save('newSpaceImage.jpg')
    newSpaceImage = imread('newSpaceImage.jpg')
    imshow('try', newSpaceImage)
    waitKey(10000)

    #code below adapted from CNURobotics at https://github.com/CNURobotics/arm_planning_2d/blob/master/roboplan.py
    '''
        Simple 2-link robot arm simulation for generating graphics for CS 471/510
        The MIT License (MIT)
        Copyright (c) 2015 David Conner (david.conner@cnu.edu)
        Permission is hereby granted, free of charge, to any person obtaining a copy
        of this software and associated documentation files (the "Software"), to deal
        in the Software without restriction, including without limitation the rights
        to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
        copies of the Software, and to permit persons to whom the Software is
        furnished to do so, subject to the following conditions:
        The above copyright notice and this permission notice shall be included in
        all copies or substantial portions of the Software.
        THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
        IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
        FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
        AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
        LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
        OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
        THE SOFTWARE.
    '''
    #!/usr/bin/env python
    # -*- coding: utf-8 -*-

    # This is the main body of the program

    # Define links for our simple robot
    #             ID   Length, color, width, parent(=None default)
    link1 = Link(" 1 ", 10.0,'b',0.5)
    link2 = Link(" 2 ", 6.0,'g',0.3, link1)

    # Define a robot with tuple of links (2 in this case but could add more)
    robot = RobotArm((link1, link2))
    robot.updateLinks((np.pi/4, np.pi)) # Define the initial angles

    obstacles = []
    r, g, b = [random.random() for i in range(3)]
    color = r, g, b, 1
    counter = 1
    # Define obstacles
    for obstacle in obstacleInfo:
        obstacles.append(obstacle)
    #obstacles.append(Obstacle("Obstacled ", (-3.0, 1.0), 2.0, 5.0, color))
    world = RobotWorld(robot, obstacles)

    #link1.updateTip(np.pi/4.0)
    #link2.updateTip(np.pi/6.0)


    # Define maximum range for our workspace plots
    max_len = (link1.length + link2.length)*1.5;


    # store angles and color of collision
    collisions_theta1=[]
    collisions_theta2=[]
    collisions_colors=[]

    # (x,y) positions of end effector in the workspace
    workspace_x=[]
    workspace_y=[]

    # Positions reachable in free C-space
    free_workspace_x=[]
    free_workspace_y=[]

    torus=None

    # Convert angles to a torus surface in 3D space
    def theta2xyz(t1,t2):
        return ( (6.*np.cos(t2)*np.cos(t1) + 10.0*np.cos(t1)),
                 (6.*np.cos(t2)*np.sin(t1) + 10.0*np.sin(t1)),
                 (6.*np.sin(t2)))

    if (True): # build C-space map

        print ("Define the torus ...")
        pi_diff = np.pi/100.0;#72.0;#36.0
        thetas = np.arange(0, np.pi+pi_diff, pi_diff)
        t1grid,t2grid = np.meshgrid(thetas,thetas)

        torus = theta2xyz(t1grid, t2grid)


        print ("Now building the C-space map ...")
        for theta1 in thetas:
            #print "theta1=",theta1
            for theta2 in thetas:

                # update the position of the links in the world
                world.updateRobotArm((theta1,theta2));

                # Store the location of the end effector
                tip = world.robot.getEndEffector();

                workspace_x.append(tip[0])
                workspace_y.append(tip[1])

                color = world.checkCollisions()
                if (color is not None):
                    collisions_theta1.append(theta1)
                    collisions_theta2.append(theta2)
                    collisions_colors.append(color)
                else:
                    if (theta1 >= 0.0): # just plot the free space on upper half of workspace
                        free_workspace_x.append(robot.links[-1].tip[0])
                        free_workspace_y.append(robot.links[-1].tip[1])


        print ("plot the C-space ...")
        fig1=plt.figure();
        ax1=fig1.gca(projection='3d');
        ax1.grid(True);
        ax1.set_xlim([-np.pi, np.pi])
        ax1.set_ylim([0, np.pi])
        ax1.set_aspect('equal', 'box');

        print ("Now plotting ", len(collisions_theta1), " collision points for C-space obstacles ...")
        ax1.scatter(collisions_theta1,collisions_theta2,c=collisions_colors,alpha=0.5,edgecolors='none')


        if (False):
          print ("Show path on C-space ...")
          Xp=[];
          Yp=[];
          for pt in pts:
            Xp.append(pt[0])
            Yp.append(pt[1])

          ax1.plot(Xp,Yp);

          ax1.plot(pts[0][0],pts[0][1],color='g',marker='o');
          ax1.plot(pts[-1][0],pts[-1][1],color='r',marker='x');

        fig1.savefig( "animation/config_space.png", format = "png", bbox_inches = 'tight', pad_inches = 0 )

        if (True):
            # Draw black (obstacle) and white image for planning
            fig1=plt.figure();
            ax1=fig1.gca();
            ax1.grid(False);
            ax1.get_xaxis().set_visible(False)
            ax1.get_yaxis().set_visible(False)
            ax1.set_xlim([-np.pi, np.pi])
            ax1.set_ylim([0, np.pi])
            ax1.set_aspect('equal', 'box');
            ax1.axis('off')
            print( "Now plotting ", len(collisions_theta1), " collision points for C-space obstacles ...")
            ax1.scatter(collisions_theta1,collisions_theta2,c='black',alpha=0.5,edgecolors='none')
            fig1.savefig( "animation/config_space_bw.png", format = "png", bbox_inches = 'tight', pad_inches = 0 )
    # Define a hand specified list of points that work for this simple environment
    pts = [(0.286, -1.04), (0.5, -2.0), (1.0, -2.125), (1.5, -2.25),
       (1.48, -1.82),  (1.42, -1.42), (1.0, -0.75), (0.75, 1.0), (0.6, 1.5),
       (0.671, 2.18), (1.5, 2.18), (1.7, 1.8), (1.87, 1.6),
       (2.012, 1.369), (2.27, 0.0), (2.48, -1.26)]

    pts = PathPlanning.fullPath() #takes obstacle info and tries to find a path for the arm
    print (pts)

    if len(pts) == 0: #prevents an exception from being thrown if no path is found
        print ("No path was found.")
        return None

    pt0 = np.asarray(deepcopy(pts[0]));# convert tuple to array so we can do math
    pt1 = np.asarray(deepcopy(pts[-1]));# convert tuple to array so we can do math

    if (np.array_equal(pt0,pt1)):
        print ("Points are equal")
    else:
        print ("Points are NOT equal")

    if (True):
        print ("Draw start ...")
        robot.updateLinks(pts[0])
        fig2=plt.figure();
        fig2.suptitle("Start")
        ax2 = fig2.gca();
        ax2.grid(True);
        ax2.set_xlim([-max_len, max_len])
        ax2.set_ylim([0, max_len])
        ax2.set_aspect('equal', 'box');

        world.drawWorld(ax2) # Draw robot and obstacles
        fig2.savefig( "animation/start_position.png", format = "png", bbox_inches = 'tight', pad_inches = 0 )


    if (True):
        print ("Draw finish ...")
        robot.updateLinks(pts[-1]) # get last point in path list

        fig3=plt.figure();
        fig3.suptitle("Finish")
        ax3 = fig3.gca();
        ax3.grid(True);
        ax3.set_xlim([-max_len, max_len])
        ax3.set_ylim([0, max_len])
        ax3.set_aspect('equal', 'box');

        world.drawWorld(ax3);

        fig3.savefig( "animation/finish_position.png", format = "png", bbox_inches = 'tight', pad_inches = 0 )


    if (True):
        print ("Draw intermediates ...")
        fig4=plt.figure();
        fig4.suptitle("Motion")
        ax4 = fig4.gca();
        ax4.grid(True);
        ax4.set_xlim([-max_len, max_len])
        ax4.set_ylim([0, max_len])
        ax4.set_aspect('equal', 'box');

        for pt in pts:
            world.updateRobotArm(pt);
            robot.drawArm(ax4)

        for obj in obstacles:
           obj.drawObstacle(ax4)
        fig4.savefig( "animation/motion_snapshots.png", format = "png", bbox_inches = 'tight', pad_inches = 0 )


    if (True):
        if (len(workspace_x) > 0): # C-space was built
            print ("Now show the workspace with ",len(workspace_x), " points, and ",len(free_workspace_x)," free points")
            fig5=plt.figure();
            fig5.suptitle("Workspace")
            ax5 = fig5.gca();
            ax5.grid(True);
            ax5.set_xlim([-max_len, max_len])
            ax5.set_ylim([0, max_len])
            ax5.set_aspect('equal', 'box');

            ax5.scatter(workspace_x,workspace_y,color='b', edgecolors='none')

            ax5.scatter(free_workspace_x,free_workspace_y,color='g',edgecolors='none')

            for obj in obstacles:
              obj.drawObstacle(ax5)
            fig5.savefig( "animation/workspace.png", format = "png", bbox_inches = 'tight', pad_inches = 0 )

        else:
            print ("No C-space map was built!")

    if (torus is not None):
        print ("Draw the torus ...")
        fig7=plt.figure()
        ax7 = fig7.add_subplot(111,projection='3d')
        ax7.plot_surface(torus[0],torus[1],torus[2])
        ax7.set_xlim([-max_len, max_len])
        ax7.set_ylim([0, max_len])
        ax7.set_zlim([-max_len, max_len])
        ax7.set_aspect('equal', 'box');
        fig7.savefig( "animation/torus.png", format = "png", bbox_inches = 'tight', pad_inches = 0 )

        if (len(collisions_theta1) > 0):
            print ("Draw the torus with collisions ...")
            fig8=plt.figure()
            ax8 = fig8.add_subplot(111,projection='3d')
            ax8.plot_wireframe(torus[0],torus[1],torus[2],alpha=0.5,rstride=4,cstride=4,color='black')
            ax8.set_xlim([-max_len, max_len])
            ax8.set_ylim([0, max_len])
            ax8.set_zlim([-max_len, max_len])
            ax8.set_aspect('equal', 'box');

            torus_objs = theta2xyz(collisions_theta1, collisions_theta2)
            ax8.scatter(torus_objs[0], torus_objs[1], torus_objs[2],
                        c=collisions_colors,alpha=0.5,edgecolors='none')

            fig8.savefig( "animation/torus_obstacles.png", format = "png", bbox_inches = 'tight', pad_inches = 0 )

    print ("Close plots to continue ...")
    plt.show()


    #print "Done! - comment out the exit below to create images for motion video"
    #sys.exit(0)

    print ("Make video of motion ...")

    fig6=plt.figure();
    fig6.suptitle("Motion")
    ax6 = fig6.gca();
    ax6.grid(True);
    ax6.set_xlim([-max_len, max_len])
    ax6.set_ylim([0, max_len])
    ax6.set_aspect('equal', 'box');

    pt0 = np.asarray(deepcopy(pts[0]) )
    robot.updateLinks(pt0);
    robot.drawArm(ax6)

    for obj in obstacles:
       obj.drawObstacle(ax6)

    cnt=0;

    fig6.savefig( "animation/roboplan_animation_0.png", format = "png", bbox_inches = 'tight', pad_inches = 0 )

    pt0 = np.asarray(deepcopy(pts[0]));# convert tuple to array so we can do math
    for pt in pts:
        pt1 = np.asarray(deepcopy(pt)) # convert tuple to array so we can do math
        print( pt1)
        if (not np.array_equal(pt1,pt0)):
          for s in np.arange(0.025,1.0,0.025): # interpolate along line from prior point to current
            pti = pt0 + (pt1-pt0)*s
            #print pti

            plt.cla()
            ax6.grid(True);
            ax6.set_xlim([-max_len, max_len])
            ax6.set_ylim([0, max_len])
            ax6.set_aspect('equal', 'box');

            world.updateRobotArm(pti);
            world.drawWorld(ax6)

            cnt = cnt +1

            fig6.savefig( "animation/roboplan_animation_"+str(cnt)+".png", format = "png", bbox_inches = 'tight', pad_inches = 0 )

        pt0 = pt1
    print ("Done!")
    return pts
#https://github.com/HebiRobotics/hebi-python-examples/blob/master/basic/05_trajectory.py
def getTheArmToMove():
    lookup = hebi.Lookup()

    sleep(2)

    points = convertObjects()
    positionsElb = []
    positionsSho = []
    for x, y in points:
        positionsElb.append(y)
        positionsSho.append(x)

    group1 = lookup.get_group_from_names(["Elbow"], ["elbow"])
    group2 = lookup.get_group_from_names(["Sholder"], ["sholder rot"])

    if group1 is None or group2 is None:
      print('Group not found! Check that the family and name of a module on the network')
      print('matches what is given in the source file.')
      exit(1)

    num_joints = group1.size + group2.size
    group_feedback1 = hebi.GroupFeedback(group1.size)
    group_feedback2 = hebi.GroupFeedback(group2.size)

    if group1.get_next_feedback(reuse_fbk=group_feedback1) is None or group2.get_next_feedback(reuse_fbk=group_feedback2) is None:
      print('Error getting feedback.')
      exit(1)

    #positions = np.zeros((num_joints, 2), dtype=np.float64)
    offset = [math.pi] * num_joints
    current_pos1 = group_feedback1.position
    current_pos2 = group_feedback2.position

    time_vector1 = []
    starttime = 0
    steps = len(positionsElb) / 60.0
    for pt in positionsElb:
        time_vector1.append((starttime))
        starttime = starttime + steps

    time_vector2 = []
    steps = len(positionsSho) / 60.0
    for pt in positionsElb:
        time_vector2.append((starttime))
        starttime = starttime + steps

    trajectory1 = hebi.trajectory.create_trajectory(time_vector1, positionsElb)
    trajectory2 = hebi.trajectory.create_trajectory(time_vector2, positionsSho)

    # Start logging in the background
    group1.start_log('logs1')
    group2.start_log('logs2')

    group_command1 = hebi.GroupCommand(group1.size)
    group_command2 = hebi.GroupCommand(group2.size)
    duration1 = trajectory1.duration

    start = time()
    t = time() - start

    while t < duration1:
      # Serves to rate limit the loop without calling sleep
      group1.get_next_feedback(reuse_fbk=group_feedback1)
      t = time() - start

      pos, vel, acc = trajectory1.get_state(t)
      group_command1.position = pos
      group_command1.velocity = vel
      group1.send_command(group_command1)

      # Serves to rate limit the loop without calling sleep
      group2.get_next_feedback(reuse_fbk=group_feedback2)
      t = time() - start

      pos, vel, acc = trajectory2.get_state(t)
      group_command2.position = pos
      group_command2.velocity = vel
      group2.send_command(group_command2)

    group1.stop_log()
    group2.stop_log()
getTheArmToMove()
exit()
