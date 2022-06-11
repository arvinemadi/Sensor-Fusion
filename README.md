# Sensor-Fusion
Algorithms and practice projects for sensor fusion project from Udacity
Original uncompleted codes available at https://github.com/udacity/SFND_Lidar_Obstacle_Detection

The complete code for Lidar detection at https://github.com/arvinemadi/SFND_Lidar_Obstacle_Detection

## Algorithms

- Segmentation:
The code shows an example implementation of RANSAC algorithm. https://en.wikipedia.org/wiki/Random_sample_consensus
The principle is to iterate for a set number of times. At each iteration picks 2 points (if 2D) or 3 points (if 3D).
Then makes the line or plane from these points and iterates over all other points and add them to a list if their distance to this line or place is less than a tolerance value.
It always keeps the largest processed line or plane and returns that. Note that we do not need to iterate over all possible combinations because of a line or plane exists picking any set of points within that gives the right result.

- Clustering:
The principle is similar to binary search. The data structure that is used is KdTree: https://en.wikipedia.org/wiki/K-d_tree
A k-d tree is created from all the cloud points. Having created a cloud a search function is created that does a binary traversal of the tree and find all the points that are within a certain tolerance value.
The clustering algorithm then does a DFS on the tree and returns different clusters.
