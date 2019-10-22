# Madalyn_Millen_Capstone_Project

Obstacle Conversion for the Robotic Configuration Space
Author Name: Madalyn Millen
Professor: Chris Kreider 

Summary: For this project, I plan to create a program that will collect information about anything detected by a camera, pass that information off to a robot. The robot will then be able to find and execute a path through a given space. Once completed, this project can be used by teachers to demonstrate to students some concepts for the basics of robotics. 

File Structure:

PathPlanning.py is the algorithm that will be used to calculate the path the robot arm will use to navigate through the given workspace.

BlobDetection.py takes a picture of the given workspace, examines that picture, and finds any obstacles. The main function, detectAndDraw(), returns a list of the contours found by the code to be used later. Since the project is not yet completed, it also draws boxes around the obstacles to show where measurements are being taken. 

ConfigSpaceConversion.py runs the detectAndDraw() code and takes the list outputted. It then converts those pixel measurements into numbers that can be used for path planning. It also calculates the safe spaces around each of the obstacles.

.idea, build, devel, and src are all catkin and ROS files, copied over from the ROS repository. These files are used as the framework for programming with ROS.

All .jpg files were either taken by me or taken by running the code in BlobDetection.py. These photos are of example workspaces with a few different layouts of obstacles to test the blob detection code.
