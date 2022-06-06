/*
As part of a Udacity sensor fusion project - the original uncomplete code from Udacity author mentioned below
Completed by Arvin.Emadi@Gmail.com
*/

/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors


// This version of the file deals with a "simulated" highwat
// This is a good way to develop and test functions and algorithms
// In this version of the file rendering, segmentation between road and obstacles, and clustering of the obstacles are tested 
//using PCL library
// In the next versions real Lidar data will be imported to do these tasks and other tasks

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
#include <unordered_map>
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = true;
    bool renderBoxOption = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Create a "simulated" lidar sensor - parameters can be adjusted in lidar.h
    Lidar* my_Lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_pointCloud = my_Lidar->scan();

    // Creating a pointProcesser object to run different functions on the point clouds
    // The pointProcessor is shown in a separate file
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;// = new ProcessPointClouds<pcl::PointXYZ>();
    

    //Option to render the simulated Rays if needed
    //renderRays(viewer, my_Lidar->position, lidar_pointCloud);

    //segmenting the road and obstacles
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> road_obstacles_cloud = pointProcessor.SegmentPlane(lidar_pointCloud, 100, 0.4);
    
    //Do the clustering on the obstacles 
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(road_obstacles_cloud.first, 1.0, 3, 30);
    
    
    //Visualize each of the clusters 
    //Option to put a bonding box around each cluster
    int clusterId = 0;
    std::vector<Color> colors = { Color(1,0,0), Color(0,1,0), Color(0,0,1) };
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);
        ++clusterId;
        //if selected using PCL library bounding box function calculate and render bonding box for each cluster
        if (renderBoxOption)
        {
            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
    }
    
    //Options for Rendering the point clouds for all points or segmented road and obstacles.
    //renderPointCloud(viewer, lidar_pointCloud, "new_cloud", Color(1, 1, 1));
    //renderPointCloud(viewer, road_obstacles_cloud.first, "obstacles", Color(1, 0, 0));
    //renderPointCloud(viewer, road_obstacles_cloud.second, "road", Color(1, 1, 1));

}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    simpleHighway(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}