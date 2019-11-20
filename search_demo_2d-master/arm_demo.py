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

import os.path
import cv2
import numpy as np
from copy import deepcopy
from russell_and_norvig_search import *
from color_map import color_map
from video_encoder import VideoEncoder
from map_loader import MapLoader

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
        self.scale = scale;

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
                    dist = infinity
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
            big_map   = cv2.resize(self.map, (0,0),fx=(1.0/self.scale),   fy=(1.0/self.scale), interpolation=cv2.INTER_NEAREST)
            big_costs = cv2.resize(self.costs, (0,0),fx=(1.0/self.scale), fy=(1.0/self.scale), interpolation=cv2.INTER_NEAREST)
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
        total_cost = cost_so_far + (self.graph.get(A,B) or infinity)
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
            return infinity

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

base_image = "config_space_bw"  # This is the base file name of the input image for map generation
map_loader.addFrame(".",base_image+".png")

scale = 0.25 # scale of map - smaller scale implies larger grid size

map_loader.createMap(scale, (-np.pi, np.pi), (-np.pi, np.pi)) # Discretize the map based on the the scaling factor

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

pts = [(0.286, -1.04), (0.5, -2.0), (1.0, -2.125), (1.5, -2.25),
       (1.48, -1.82),  (1.42, -1.42), (1.0, -0.75), (0.75, 1.0), (0.6, 1.5),
       (0.671, 2.18), (1.5, 2.18), (1.7, 1.8), (1.87, 1.6),
       (2.012, 1.369), (2.27, 0.0), (2.48, -1.16)]

start= map_loader.gridPoint(pts[0])
goal = map_loader.gridPoint(pts[-1])

print "Start=",start
print "Goal =",goal


# Define the test cases we want to run
tests = [#("depth_first_",  depth_first_graph_search),
         #("breadth_first_",breadth_first_search),
         #("uniform_cost_", uniform_cost_search),
         #("astar_search_euclid_",    astar_search,0),
         ("astar_search_euclid2_",   astar_search,4)] #,
         #("astar_search_euclid3_",   astar_search,5),
         #("astar_search_euclid025_", astar_search,6),
         #("astar_search_euclid05_",  astar_search,7)]#,
         #("astar_search_dx_",        astar_search,1),
         #("astar_search_dy_",        astar_search,2),
         #("astar_search_manhattan_", astar_search,3),
         #("greedy_search_euclid_",   greedy_best_first_graph_search,0),
         #("greedy_search_dx_",       greedy_best_first_graph_search,1),
         #("greedy_search_dy_",       greedy_best_first_graph_search,2),
         #("greedy_search_manhattan_",greedy_best_first_graph_search,3)   ]
for test in tests:
    print "Set up the "+test[0]+" ..."
    file_name = target_dir+"/"+test[0]+base_image
    video_encoder = VideoEncoder(file_name, map_loader.map, frame_rate = 60.0, fps_factor=1.0, comp_height=1.0/scale, comp_width=2.0/scale)

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

print "Done! - press return to exit"
cv2.waitKey(5000)
