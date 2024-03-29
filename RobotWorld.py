#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
  Classes for a simple kinematic robot arm simulation
  This code used matplotlib for generating graphics for CS 471/510.
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

import numpy as np
from copy import deepcopy
from mpl_toolkits.mplot3d import Axes3D

import matplotlib.pyplot as plt
import matplotlib.patches as patches

import sys

class Link:

    def __init__(self, name, length, color='b', width=0.5, parent=None):
        self.name = name;

        self.current_image = -1
        self.length = length;
        self.color = color;
        self.width = width;
        self.angle       = 0.0;
        self.total_angle = 0.0;
        self.base        = (0.0, 0.0)
        self.parent = parent;
        self.total_angle = self.total_angle + self.angle;
        self.tip = (self.base[0] + self.length*np.cos(self.total_angle),
                    self.base[0] + self.length*np.sin(self.total_angle));
        self.normal = (-np.sin(self.total_angle),np.cos(self.total_angle))

        #print "link ",self.name," base=",self.base," tip=",self.tip


    def updateTip(self, angle):
        self.angle = deepcopy(angle);

        if (self.parent is not None):
            self.base        = deepcopy(self.parent.tip)
            self.total_angle = deepcopy(self.parent.total_angle);
        else:
            self.base        = (0.0, 0.0);
            self.total_angle = 0.0;

        self.total_angle = self.total_angle + self.angle;

        #print " Update link ",self.name," base = ",self.base, " angle=",self.angle," total angle=",self.total_angle

        self.tip = (self.base[0] + self.length*np.cos(self.total_angle),
                    self.base[1] + self.length*np.sin(self.total_angle));
        #print "link ",self.name," angle=",self.angle," base=",self.base," tip=",self.tip


        # Define vector normal to the link (for later distance calculation)
        self.normal = (-np.sin(self.total_angle), np.cos(self.total_angle));

    def drawLink(self, ax):
        #print "draw link ",self.name," angle=",self.angle," base=",self.base," tip=",self.tip

        #surf = ax2.plot(( self.base[0], self.tip[0]), (self.base[1], self.tip[1]),
        #                linewidth=self.width, color=self.color);

        center = ((self.base[0]+self.tip[0])/2.0, (self.base[1]+self.tip[1])/2.0)
        corner = ( (self.base[0] - self.normal[0]*self.width*0.5),
                   (self.base[1] - self.normal[1]*self.width*0.5) )

        ax.add_patch(patches.Rectangle(corner,self.length, self.width, angle=(self.total_angle*180.0/np.pi), color=self.color));


    def drawTip(self, ax):
        ax.plot(self.tip[0], self.tip[1],color=self.color);


class RobotArm:
    def __init__(self, links):

        self.links = links;

    # Update the link positions given angles for each link relative its parent link
    # Assuming processing from fixed base up to final child link
    def updateLinks(self, angles):

        if (len(angles) != len(self.links)):
            print "Invalid angles! - need one per link";
            sys.exit(-1)

        # Assuming list of angles with one angle for each link
        for link,angle in enumerate(angles):
            self.links[link].updateTip(angle)

    # Simple collision checking for circular obstacles
    # Returns None for collision free, or a color for object that cause the collision
    def checkCollisions(self, objects):

        collision = False;

        # Check all objects
        for obj in objects:

            #print "object ",obj.name

            # Against all links
            for link in self.links:

                # Vector from link base to object lowerleft
                dX = ( (obj.lowerLeft[0] - link.base[0]), (obj.lowerLeft[1] - link.base[1]) )

                dist = dX[0]*link.normal[0] + dX[1]*link.normal[1]
                val = np.fabs(dist)
                # check distance from obstacle lowerLeft to line along a link
                if (val < (link.width/2.0 + obj.width/2.0)):
                    #print "potential collision"

                    # Calculate the projection of vector to obstacle along the link
                    dY = (dX[0] - dist*link.normal[0], dX[1]-dist*link.normal[1])

                    # Distance along the link
                    proj = dY[0]*np.cos(link.total_angle) + dY[1]*np.sin(link.total_angle)

                    if ((proj >= 0.0) and (proj <= (link.length+obj.width))):
                        print " Collision between link ",link.name," and obstacle ",obj.name, " at dist=",dist, " proj=",proj
                        return obj.color; # return the first collision encountered

        return None;


    # Draw each arm
    def drawArm(self, ax):
        for link in self.links:
            link.drawLink(ax)

    # Draw the final end effector position
    def drawEE(self, ax):
       self.links[-1].drawTip(ax)

    # Return the position (x,y) of the end effector (tip of last link)
    def getEndEffector(self):
            return self.links[-1].tip;

# Simple rectangular obstacle
class Obstacle:

    def __init__(self, name, lowerLeft, width, height, color, angle=0.0):
        self.name = name
        self.lowerLeft = lowerLeft
        self.width = width
        self.height = height
        self.color = color
        self.angle = angle

    def drawObstacle(self,ax2):
        ax2.add_patch(patches.Rectangle(self.lowerLeft,self.width, self.height, angle=self.angle, color=self.color));


# Class to hold the robot and obstacles
class RobotWorld:

    def __init__(self, robot, obstacles):
        self.robot      = robot;
        self.obstacles  = obstacles;

    # Draw all the obstacles on the axis
    def drawObstacles(self,ax2):
        for obj in self.obstacles:
            obj.drawObstacle(ax2);

    # Draw a small patch for the robot end effector
    def drawRobotEndEffector(self, ax):
       self.robot.links[-1].drawTip(ax)

    # Draw the robot
    def drawRobot(self, ax):
        self.robot.drawArm(ax);

    # Draw both robot arm and obstacles
    def drawWorld(self, ax):
        self.drawRobot(ax);
        self.drawObstacles(ax);

    # Update the angles of the robot links
    def updateRobotArm(self, angles):
        return self.robot.updateLinks(angles);

    # Check for collision between current robot arm position and the obstacles
    def checkCollisions(self):
        return self.robot.checkCollisions(self.obstacles)