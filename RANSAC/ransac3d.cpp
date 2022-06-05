//Original author is referenced below - this has been posted as an excercie on Udacity to complete and get more feel on hyper-parameters for RANSAC algorithm
//completion, modifications and updates done by Arvin.Emadi@Gmail.com

/* \author Aaron Brown */

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"


//This functions simulates a 2D data to test the algorithm - first a line data is created and noise is added to that.. then some random outlier are added
//The number of points are small but can be adjusted
pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
    //comment new
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

//This functions returns data from a file

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
    std::cout << "iteration number is " << maxIterations << endl;
    while (maxIterations > 0) {
        maxIterations--;
        std::cout << "iteration number is " << maxIterations << endl;
        // Randomly sample subset and fit line
        std::unordered_set<int> iteration_inliers;
        
        //creat two random index values
        while (iteration_inliers.size() < 2)
            iteration_inliers.insert(rand() % cloud->points.size());

        //pick the two randomly selected points
        float x1, y1, x2, y2;
        auto it = iteration_inliers.begin();
        x1 = cloud->points[*it].x;
        y1 = cloud->points[*it].y;
        it++;
        x2 = cloud->points[*it].x;
        y2 = cloud->points[*it].y;
        
        //find the coefficients of the line between these two random points
        float A = y1 - y2;
        float B = x2 - x1;
        float C = x1 * y2 - y1 * x2;

        //Now go over all the points and calculate their distance, if smaller than the tolerance add them
        for (int i = 0; i < cloud->points.size(); i++)
        {
            if (iteration_inliers.find(i) != iteration_inliers.end())
                continue;

            // pick a unique point at index i
            float x3 = cloud->points[i].x;
            float y3 = cloud->points[i].y;
            
            // Measure distance between every point and fitted line
            // if a line is Ax + By + C = 0, the distance of point x, y to this line is |Ax+By+C|/sqrt(A^2 + B^2)
            float dist = fabs(x3 * A + y3 * B + C) / sqrt(A * A + B * B);

            // If distance is smaller than threshold count it as inlier
            if (dist <= distanceTol)
                iteration_inliers.insert(i);
        }

        if (iteration_inliers.size() > inliersResult.size())
            inliersResult = iteration_inliers;

    }
	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 10, 1.0);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
