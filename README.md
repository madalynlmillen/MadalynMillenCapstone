# Madalyn_Millen_Capstone_Project

Obstacle Conversion for the Robotic Configuration Space
Author Name: Madalyn Millen
Professor: Chris Kreider 

Summary: For this project, I plan to create a program that will collect information about anything detected by a camera, pass that information off to a robot. The robot will then be able to find and execute a path through a given space. Once completed, this project can be used by teachers to demonstrate to students some concepts for the basics of robotics. 

File Structure:
The animation folder holds all the images created by ConfigSpaceConversion, which includes the configuration space image and a simulation of the arm following its path. These images can be made into a video using the .avi extension.

The output folder are images, mp4s, and text files outputted by PathPlanning, used to create the videos of the path planning algorithms searching for their answers.

The search_demo_2d-master folder folds all the helper functions used by PathPlanning to create the output file and to actually run the searches. The copyright for these files is as follows:

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
The src folder holds extra functions used by ROS to carry out variuos actions.

The vision_opencv-melodic folder holds extra functions used to convert the images used from pixel to 3d coordinates. It's functionality has not yet been integrated into the rest of the code. It's copyright message is as follows:

PathPlanning.py is the set of algorithms that are used to calculate the path the robot arm will use to navigate through the given workspace. Several types of algorithms are tested, including depth-first, breadth-first, A*, and uniform cost. The points collected are then converted into radians then passed to ConfigSpaceConversion to be used for the robot and the simulation.

BlobDetection.py takes a picture of the given workspace, examines that picture, and finds any obstacles. The main function, detectAndDraw(), returns a list of the contours found by the code to be used later. It also draws boxes around the obstacles to show where measurements are being taken. The rest of the functions are used to create the obstacle objects used by ConfigSpaceConversion and PathPlanning to create the configuration space image.

ConfigSpaceConversion.py runs the detectAndDraw() code and takes the list outputted. It then converts those pixel measurements into numbers that can be used for path planning. The configuration space image is created then passed to PathPlanning so it can create the path needed. This code also contains the pieces of HEBI code needed to pass the radian points to the arm directly. 

.idea, build, devel, and src are all catkin and ROS files, copied over from the ROS repository. These files are used as the framework for programming with ROS. In order to make use of these files, in a separate terminal, run the command roscore. If need be, run the command catkin_make afterwards to make sure all files are updated.

All .jpg files were either taken by me or taken by running the code in BlobDetection.py. These photos are of example workspaces with a few different layouts of obstacles to test the blob detection code.
