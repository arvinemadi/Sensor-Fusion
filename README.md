# Sensor-Fusion
Algorithms and practice projects for sensor fusion project from Udacity
Original uncompleted codes available at https://github.com/udacity/SFND_Lidar_Obstacle_Detection

## Lidar Simulation
The folder has three files.
Environment creates a simulate highway and calls a simulated Lidar. The parameters of the lidar are in lidar.h
The created lidar object returns a cloud of points. This cloud of points include the points from the road and obstacles (like cars)
pointProcesser has several functions to apply to the data points. The functions use PCL library that needs to be installed to call these functions.

Two main algorithms are applied to the cloud data:

- ### Segmentation
The first task is to apply a segmentation algorithm to the cloud of all points and take the road points out of it. 
Then we will have two separate clouds, one that shows the road, and the other one that shows all the obstacles
The algorithm to do this segmentation is based on RANSAC. There is separate folder on that with more details.

- ### Clustering
After we have all the obstacle points we will separate them into different clusters. The idea is that each cluster is showing a different object or a group of them that are close to each other.
The algorithm for this is based on KdTree, There is separate folder on that with more details

## Algorithms

- Segmentation:
The code shows an example implementation of RANSAC algorithm. https://en.wikipedia.org/wiki/Random_sample_consensus
The principle of algorithm is greedy. It iterates for set number of times. At each iteration picks 2 points (if 2D) or 3 points (if 3D).
Then makes the line or place from these points and iterates over all other points and add them to a list if their distance to this line or place is less than a tolerance value.
It always keeps the largest processed line or plane and returns that. Note that we do not need to iterate over all possible combinations because of a line or plane exists picking any set of points within that gives the right result.

- Clustering:
The principle is similar to binary search. The data structure that is used is KdTree: https://en.wikipedia.org/wiki/K-d_tree
A k-d tree is created from all the cloud points. Having created a cloud a search function is created that does a binary traversal of the tree and find all the points that are within a certain tolerance value.
The clustering algorithm then does a DFS on the tree and returns different clusters.