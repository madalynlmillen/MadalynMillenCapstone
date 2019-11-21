import os.path
from copy import deepcopy
from mpl_toolkits.mplot3d import Axes3D
from cv2 import *
import ConfigSpaceConversion
import matplotlib.pyplot as plt
import sys
import numpy as np
from RobotWorld import *
import os.path
from copy import deepcopy
sys.path.append("C:\Users\madal\PycharmProjects\MadalynMillenCapstone\search_demo_2d-master")
from copy import deepcopy
from russell_and_norvig_search import *
from color_map import color_map
from video_encoder import VideoEncoder
from map_loader import MapLoader

_, obstacleInfo = ConfigSpaceConversion.convertObjects()

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
robot.updateLinks((np.pi/4.0, -np.pi/6.0)) # Define the initial angles

obstacles = []
counter = 1
# Define obstacles
for obstacle in obstacleInfo:
    obstacles.append(obstacle)
world = RobotWorld(robot, obstacles)

#link1.updateTip(np.pi/4.0)
#link2.updateTip(np.pi/6.0)


# Define maximum range for our workspace plots
max_len = (link1.length + link2.length)*1.5;


# Define a hand specified list of points that work for this simple environment
pts = [(0.286, -1.04), (0.5, -2.0), (1.0, -2.125), (1.5, -2.25),
       (1.48, -1.82),  (1.42, -1.42), (1.0, -0.75), (0.75, 1.0), (0.6, 1.5),
       (0.671, 2.18), (1.5, 2.18), (1.7, 1.8), (1.87, 1.6),
       (2.012, 1.369), (2.27, 0.0), (2.48, -1.26)]

pt0 = np.asarray(deepcopy(pts[0]));# convert tuple to array so we can do math
pt1 = np.asarray(deepcopy(pts[-1]));# convert tuple to array so we can do math

if (np.array_equal(pt0,pt1)):
    print "Points are equal"
else:
    print "Points are NOT equal"


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

    print "Define the torus ..."
    pi_diff = np.pi/100.0;#72.0;#36.0
    thetas = np.arange(-np.pi, np.pi+pi_diff, pi_diff)
    t1grid,t2grid = np.meshgrid(thetas,thetas)

    torus = theta2xyz(t1grid, t2grid)


    print "Now building the C-space map ..."
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


    print "plot the C-space ..."
    fig1=plt.figure();
    ax1=fig1.gca();
    ax1.grid(True);
    ax1.set_xlim([-np.pi, np.pi])
    ax1.set_ylim([-np.pi, np.pi])
    ax1.set_aspect('equal', 'box');

    print "Now plotting ", len(collisions_theta1), " collision points for C-space obstacles ..."
    ax1.scatter(collisions_theta1,collisions_theta2,c=collisions_colors,alpha=0.5,edgecolors='none')


    if (False):
      print "Show path on C-space ..."
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
        ax1.set_ylim([-np.pi, np.pi])
        ax1.set_aspect('equal', 'box');
        ax1.axis('off')
        print "Now plotting ", len(collisions_theta1), " collision points for C-space obstacles ..."
        ax1.scatter(collisions_theta1,collisions_theta2,c='black',alpha=0.5,edgecolors='none')
        fig1.savefig( "animation/config_space_bw.png", format = "png", bbox_inches = 'tight', pad_inches = 0 )


if (True):
    print "Draw start ..."
    robot.updateLinks(pts[0])
    fig2=plt.figure();
    fig2.suptitle("Start")
    ax2 = fig2.gca();
    ax2.grid(True);
    ax2.set_xlim([-max_len, max_len])
    ax2.set_ylim([-max_len, max_len])
    ax2.set_aspect('equal', 'box');

    world.drawWorld(ax2) # Draw robot and obstacles
    fig2.savefig( "animation/start_position.png", format = "png", bbox_inches = 'tight', pad_inches = 0 )


if (True):
    print "Draw finish ..."
    robot.updateLinks(pts[-1]) # get last point in path list

    fig3=plt.figure();
    fig3.suptitle("Finish")
    ax3 = fig3.gca();
    ax3.grid(True);
    ax3.set_xlim([-max_len, max_len])
    ax3.set_ylim([-max_len, max_len])
    ax3.set_aspect('equal', 'box');

    world.drawWorld(ax3);

    fig3.savefig( "animation/finish_position.png", format = "png", bbox_inches = 'tight', pad_inches = 0 )


if (True):
    print "Draw intermediates ..."
    fig4=plt.figure();
    fig4.suptitle("Motion")
    ax4 = fig4.gca();
    ax4.grid(True);
    ax4.set_xlim([-max_len, max_len])
    ax4.set_ylim([-max_len, max_len])
    ax4.set_aspect('equal', 'box');

    for pt in pts:
        world.updateRobotArm(pt);
        robot.drawArm(ax4)

    for obj in obstacles:
       obj.drawObstacle(ax4)
    fig4.savefig( "animation/motion_snapshots.png", format = "png", bbox_inches = 'tight', pad_inches = 0 )


if (True):
    if (len(workspace_x) > 0): # C-space was built
        print "Now show the workspace with ",len(workspace_x), " points, and ",len(free_workspace_x)," free points"
        fig5=plt.figure();
        fig5.suptitle("Workspace")
        ax5 = fig5.gca();
        ax5.grid(True);
        ax5.set_xlim([-max_len, max_len])
        ax5.set_ylim([-max_len, max_len])
        ax5.set_aspect('equal', 'box');

        ax5.scatter(workspace_x,workspace_y,color='b', edgecolors='none')

        ax5.scatter(free_workspace_x,free_workspace_y,color='g',edgecolors='none')

        for obj in obstacles:
          obj.drawObstacle(ax5)
        fig5.savefig( "animation/workspace.png", format = "png", bbox_inches = 'tight', pad_inches = 0 )

    else:
        print "No C-space map was built!"

if (torus is not None):
    print "Draw the torus ..."
    fig7=plt.figure()
    ax7 = fig7.add_subplot(111,projection='3d')
    ax7.plot_surface(torus[0],torus[1],torus[2])
    ax7.set_xlim([-max_len, max_len])
    ax7.set_ylim([-max_len, max_len])
    ax7.set_zlim([-max_len, max_len])
    ax7.set_aspect('equal', 'box');
    fig7.savefig( "animation/torus.png", format = "png", bbox_inches = 'tight', pad_inches = 0 )

    if (len(collisions_theta1) > 0):
        print "Draw the torus with collisions ..."
        fig8=plt.figure()
        ax8 = fig8.add_subplot(111,projection='3d')
        ax8.plot_wireframe(torus[0],torus[1],torus[2],alpha=0.5,rstride=4,cstride=4,color='black')
        ax8.set_xlim([-max_len, max_len])
        ax8.set_ylim([-max_len, max_len])
        ax8.set_zlim([-max_len, max_len])
        ax8.set_aspect('equal', 'box');

        torus_objs = theta2xyz(collisions_theta1, collisions_theta2)
        ax8.scatter(torus_objs[0], torus_objs[1], torus_objs[2],
                    c=collisions_colors,alpha=0.5,edgecolors='none')

        fig8.savefig( "animation/torus_obstacles.png", format = "png", bbox_inches = 'tight', pad_inches = 0 )

print "Close plots to continue ..."
plt.show()


#print "Done! - comment out the exit below to create images for motion video"
#sys.exit(0)

print "Make video of motion ..."

fig6=plt.figure();
fig6.suptitle("Motion")
ax6 = fig6.gca();
ax6.grid(True);
ax6.set_xlim([-max_len, max_len])
ax6.set_ylim([-max_len, max_len])
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
    print pt1
    if (not np.array_equal(pt1,pt0)):
      for s in np.arange(0.025,1.0,0.025): # interpolate along line from prior point to current
        pti = pt0 + (pt1-pt0)*s
        #print pti

        plt.cla()
        ax6.grid(True);
        ax6.set_xlim([-max_len, max_len])
        ax6.set_ylim([-max_len, max_len])
        ax6.set_aspect('equal', 'box');

        world.updateRobotArm(pti);
        world.drawWorld(ax6)

        cnt = cnt +1

        fig6.savefig( "animation/roboplan_animation_"+str(cnt)+".png", format = "png", bbox_inches = 'tight', pad_inches = 0 )

    pt0 = pt1

print "Done!"

#!/usr/bin/python

'''

     Search demonstrations using Python code from AI:MA by Russell and Norvig
           https://code.google.com/p/aima-python/



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

class GridProblem(Problem):
    "The problem of searching a grid from one node to another."
    def __init__(self, initial, goal, graph, map, scale, video_encoder = None):
        Problem.__init__(self, initial, goal)
        self.graph = graph
        self.map   = deepcopy(map)
        self.costs = np.ones(map.shape,dtype=np.uint8)*255
        self.cost_values = np.ones(map.shape[:2],dtype=np.uint8)*255

        self.expansion = 0

        print "map = ",self.map.shape
        print "scale = ",scale

        # Define a video encoder to generate videos during the search process
        self.video_encoder =  video_encoder;

    # Define what actions are permissible in this world
    def actions(self, A):
        "The actions at a graph node are just its neighbors."
        self.expansion += 1
        x=A[0]
        y=A[1]
        #print "Node (",x,", ",y,")"
        if (self.map[x][y][0] < 200):
            self.map[x][y][0] = 255
            #self.map[x][y][1] = 255
            #self.map[x][y][2] = 255
        #actions = []

        # 8 connected grid world
        for i in range(-1,2):
            for j in range(-1,2):
                if (i == 0 and j == 0): continue; # self reference

                # Convert to global grid coordinates
                ix = x + i
                iy = y + j

                # Keep it in bounds (assumes bounded world)
                if (ix < 0): continue;
                if (iy < 0): continue;
                if (ix >= self.map.shape[0]): continue;
                if (iy >= self.map.shape[1]): continue;

                # Connect this node in the graph
                B = (ix, iy)
                dist = abs(2*i)+abs(2*j)
                if (dist > 3):
                    dist = 3  # approximate sqrt(2)

                #print "Blocked at ",ix,", ",iy, " = ",self.map[ix][iy]

                if (self.map[ix][iy][2] > 190):
                    dist = float('inf')
                    #print "Blocked at ",ix,", ",iy
                    #cv2.waitKey()
                else:
                  if (self.map[ix][iy][2] > 0):
                    # Multply distance by a cost function based on red channel color in map (for varying costs)
                    dist *= (5.01 + (float(self.map[ix][iy][2])/8.))
                    dist = int(dist)

                  if (self.map[ix][iy][0] < 100):
                    self.map[ix][iy][0] = 92 # mark the cells we have visted during search

                  #print "   Adding ",B," at d=",dist
                  self.graph.connect(A,B,dist)  # add valid edge to the graph according to search criteria
                  #actions.append((ix,iy))
                    #print " Keys for A=",A," = ", self.graph.get(A).keys()


        if (self.video_encoder is not None):
            # Add video to encode the search behavior
            #print "actions=",actions
            scale=0.1
            big_map   = cv2.resize(self.map, (0,0),fx=(1.0/scale), fy=(1.0/scale), interpolation=cv2.INTER_NEAREST)
            big_costs = cv2.resize(self.costs, (0,0),fx=(1.0/scale), fy=(1.0/scale), interpolation=cv2.INTER_NEAREST)
            cv2.imshow("Search ",big_map)
            cv2.imshow("Costs ", big_costs)
            cv2.waitKey(25)
            self.video_encoder.addDualFrame(big_map, big_costs)

        return self.graph.get(A).keys()

    def result(self, state, action):
        "The result of going to a neighbor is just that neighbor."
        return action

    # Update the path cost and color the map image
    def path_cost(self, cost_so_far, A, action, B):
        #print "A",A," to B",B," = ", self.graph.get(A,B)," + ", cost_so_far
        total_cost = cost_so_far + (self.graph.get(A,B) or float('inf'))
        if (self.cost_values[B[0]][B[1]] > total_cost):
            value = total_cost
            if (value > 255):
                value = 255
            color = color_map(value)
            #print color
            self.costs[B[0]][B[1]][0] = color[0]*255
            self.costs[B[0]][B[1]][1] = color[1]*255
            self.costs[B[0]][B[1]][2] = color[2]*255

        return total_cost

    def total_path_cost(self,path):
        prior_node = path[0]
        cost_so_far = 0
        for next_node in path[1:]:
            #print "A",prior_node.state, " to B",next_node.state," = ", cost_so_far, " + ", self.graph.get(prior_node.state, next_node.state)

            cost_so_far += self.graph.get(prior_node.state, next_node.state)#self.path_cost(cost_so_far,prior_node.state, None, next_node.state)
            prior_node = next_node
        return cost_so_far

    # Now define the different heuristics we will use during this search
    #
    def h(self, node):
        "h function is straight-line distance from a node's state to goal."
        locs = getattr(self.graph, 'locations', None)
        if locs:
            return int(distance(locs[node.state], locs[self.goal]))
        else:
            return float('inf')

    def h_x_distance(self,node):
        return abs(node.state[0] - self.goal[0])

    def h_y_distance(self,node):
        return abs(node.state[1] - self.goal[1])

    def h_manhattan(self,node):
        return self.h_x_distance(node) + self.h_y_distance(node)

    def h_euclid(self,node):
        dx = self.h_x_distance(node)
        dy = self.h_y_distance(node)
        dist = np.sqrt(dx*dx + dy*dy)
        return int(dist)

    def h_euclid2(self,node):
        dx = self.h_x_distance(node)
        dy = self.h_y_distance(node)
        dist = 2*np.sqrt(dx*dx + dy*dy)
        return int(dist)

    def h_euclid3(self,node):
        dx = self.h_x_distance(node)
        dy = self.h_y_distance(node)
        dist = 3*np.sqrt(dx*dx + dy*dy)
        return int(dist)

    def h_euclid025(self,node):
        dx = self.h_x_distance(node)
        dy = self.h_y_distance(node)
        dist = 0.25*np.sqrt(dx*dx + dy*dy)
        return int(dist)

    def h_euclid05(self,node):
        dx = self.h_x_distance(node)
        dy = self.h_y_distance(node)
        dist = 0.5*np.sqrt(dx*dx + dy*dy)
        return int(dist)


# This is the main part of the demo program
map_loader = MapLoader() # Create a MapLoader to load the world map from a simple image

base_image = "simple"  # This is the base file name of the input image for map generation
map_loader.addFrame(".",base_image+".png")

scale = 0.1
map_loader.createMap(scale) # Discretize the map based on the the scaling factor

# Create a big version of discretized map for better visualization
big_map = cv2.resize(map_loader.map, (0,0),fx=(1.0/scale), fy=(1.0/scale), interpolation=cv2.INTER_NEAREST)

cv2.imshow("Image",map_loader.image)
cv2.imshow("Map",  map_loader.map)
cv2.imshow("Big",  big_map)

target_dir = "output"
if not os.path.exists(target_dir):
    print "Creating target directory <",target_dir,"> ..."
    try:   os.makedirs(target_dir)
    except:
        print "Failed to create target path!"
        exit()

print "Writing the base images ..."
cv2.imwrite(target_dir+"/"+base_image+"_img.png",map_loader.image)
cv2.imwrite(target_dir+"/"+base_image+"_map.png",map_loader.map)
cv2.imwrite(target_dir+"/"+base_image+"_big_map.png",big_map)

print "Wait for key input..."
#cv2.waitKey()


print "Doing the search ..."
grid = UndirectedGraph()  # Using Russell and Norvig code

start=(4,4)
goal=(28,24)


# Define the test cases we want to run
tests = [("depth_first_",  depth_first_graph_search),
         ("breadth_first_",breadth_first_search),
         ("uniform_cost_", uniform_cost_search),
         ("astar_search_euclid_",    astar_search,0),
         ("astar_search_euclid2_",   astar_search,4),
         ("astar_search_euclid3_",   astar_search,5),
         ("astar_search_euclid025_", astar_search,6),
         ("astar_search_euclid05_",  astar_search,7),
         ("astar_search_dx_",        astar_search,1),
         ("astar_search_dy_",        astar_search,2),
         ("astar_search_manhattan_", astar_search,3),
         ("greedy_search_euclid_",   greedy_best_first_graph_search,0),
         ("greedy_search_dx_",       greedy_best_first_graph_search,1),
         ("greedy_search_dy_",       greedy_best_first_graph_search,2),
         ("greedy_search_manhattan_",greedy_best_first_graph_search,3)   ]
for test in tests:
    print "Set up the "+test[0]+" ..."
    file_name = target_dir+"/"+test[0]+base_image
    video_encoder = VideoEncoder(file_name, map_loader.map, frame_rate = 30.0, fps_factor=1.0, comp_height=1.0/scale, comp_width=2.0/scale)

    print "     output to ",file_name
    problem2 = GridProblem(start, goal, grid, map_loader.map,scale,video_encoder)

    # Load the correct grid search algorithm and heuristics
    print "------------- call ---------------------"
    if (len(test) > 2):
        if (test[2] == 0):
           result, max_frontier_size = test[1](problem2, problem2.h_euclid)
        #
        elif (test[2] == 1):
           result, max_frontier_size = test[1](problem2, problem2.h_x_distance)
        #
        elif (test[2] == 2):
           result, max_frontier_size = test[1](problem2, problem2.h_y_distance)
        #
        elif (test[2] == 3):
           result, max_frontier_size = test[1](problem2, problem2.h_manhattan)
        #
        elif (test[2] == 4):
           result, max_frontier_size = test[1](problem2, problem2.h_euclid2)
        #
        elif (test[2] == 5):
           result, max_frontier_size = test[1](problem2, problem2.h_euclid3)
        #
        elif (test[2] == 6):
           result, max_frontier_size = test[1](problem2, problem2.h_euclid025)
        #
        elif (test[2] == 7):
           result, max_frontier_size = test[1](problem2, problem2.h_euclid05)
        #
        else:
           print "Help",test[2]
    else:
       result, max_frontier_size = test[1](problem2)
    #result,max_frontier_size=depth_first_graph_search(problem2)
    print "-------------return---------------------"


    #result = depth_first_graph_search(problem2)
    #result = breadth_first_search(problem2)
    #result = uniform_cost_search(problem2)
    #@result = astar_search(problem2, h=problem2.h_euclid)#manhattan)#y_distance)
    ftxt = open(file_name+'.txt','w')
    print "     Result=",result
    print "     expansions = ",problem2.expansion
    ftxt.write("expansions = "+str(problem2.expansion)+"\n")
    ftxt.write("max frontier = "+str(max_frontier_size)+"\n")
    if (result is not None):
       path = result.path()
       ftxt.write("path cost="+str(problem2.total_path_cost(path))+"\n")
       ftxt.write("Path="+str(path)+"\n")
       print "path cost=",problem2.total_path_cost(path)
       print "Path=",path
       print "Plotting path ..."
       map_loader.plotPath(path, 1.0)# scale)
       big_path = cv2.resize(map_loader.path, (0,0),fx=(1.0/scale), fy=(1.0/scale), interpolation=cv2.INTER_LINEAR)
       cv2.imshow("Path",big_path)
       cv2.imwrite(file_name+"_path.png",big_path)
    else:
        ftxt.write('no path!')
    ftxt.close()

    print "     Close the video ..."
    problem2.video_encoder.release()


    cv2.waitKey(500)
