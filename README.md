# Sensor-Fusion
Algorithms and practice projects for sensor fusion project from Udacity
Original uncompleted codes available at https://github.com/udacity/SFND_Lidar_Obstacle_Detection

## Lidar Simulation
Folder has three files.
Environment creates a simulate highway and calls a simulated Lidar. The parameters of the lidar are in lidar.h
The created lidar object returns a cloud of points. This cloud of points include the points from the road and obstacles (like cars)
Two main algorithms are applied to the cloud data:

- ### Segmentation
The first task is to apply a segmentation algorithm to the cloud of all points and take the road points out of it. 
Then we will have two separate clouds, one that shows the road, and the other one that shows all the obstacles
The algorithm to do this segmentation is based on RANSAC. There is separate folder on that with more details.

- ### Clustering
After we have all the obstacle points we will separate them into different clusters. The idea is that each cluster is showing a different object or a group of them that are close to each other.
The algorithm for this is based on KdTree, There is separate folder on that with more details
