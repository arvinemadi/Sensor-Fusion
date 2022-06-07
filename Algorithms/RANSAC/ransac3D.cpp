//Original author is referenced below - this has been posted as an excercie on Udacity to complete and get more feel on hyper-parameters for RANSAC algorithm
//completion, modifications and updates done by Arvin.Emadi@Gmail.com

/* \author Aaron Brown */
// using templates for processPointClouds so also include .cpp to help linker
#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
#include "../../processPointClouds.cpp"

//This functions loads returns data from a file for debug and visualization - real data comes from a stream
pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("C:/Users/HP/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/simpleHighway.pcd");
}

//Initializing PCL viewer
pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

//Main function of the RANSAC algorithm
std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;  // the biggest set will be always stored here during iterations
    srand(time(NULL));                      // keeping track of time

    // For # iterations of iterations pick 3 points randomly - make a plane with them - add any other point that is within distance tolerance to the plane
    // At the end of each iteration check if the current plane is bigger that previous max plane and if yes replace that with current plane
    while (maxIterations > 0) {
        maxIterations--;
        std::cout << "iteration number is " << maxIterations << endl;
        
        // this set will be plane of each iteration
        std::unordered_set<int> iteration_inliers;

        //creat three random unique index values
        while (iteration_inliers.size() < 3)
            iteration_inliers.insert(rand() % cloud->points.size());

        //pick the x,y,z of the the three randomly selected points
        float x1, y1, z1, x2, y2, z2, x3, y3, z3;

        auto it = iteration_inliers.begin();
        x1 = cloud->points[*it].x;
        y1 = cloud->points[*it].y;
        z1 = cloud->points[*it].z;

        it++;
        x2 = cloud->points[*it].x;
        y2 = cloud->points[*it].y;
        z2 = cloud->points[*it].z;

        it++;
        x3 = cloud->points[*it].x;
        y3 = cloud->points[*it].y;
        z3 = cloud->points[*it].z;

        //find the coefficients of the line between these two random points
        //for 3 points of p1(x1,y1,z1), p2(x2,y2,z2), p3(x3,y3,z3) if we define <i,j,k> = (p2 - p1) X (p3 - p1) where X is cross-product
        //Then the coefficients of the plane Ax + By + Cz + D = 0 that fits to this line is:
        // A = i
        // B = j
        // C = k
        // D = -(ix1 + jy1 + kz1)

        float A = (y2 - y1) * (z3 - z1) - (y3 - y1) * (z2 - z1);
        float B = (x2 - x1) * (z3 - z1) - (x3 - x1) * (z2 - z1);
        float C = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1);
        float D = -(A * x1 + B * y1 + C * z1);

        //Now go over all the points and calculate their distance, if smaller than the tolerance add them
        for (int i = 0; i < cloud->points.size(); i++)
        {
            if (iteration_inliers.find(i) != iteration_inliers.end())
                continue;

            // pick the co-ordinates of the unique point at index i
            float x4 = cloud->points[i].x;
            float y4 = cloud->points[i].y;
            float z4 = cloud->points[i].z;

            // Measure distance between the point and the fitted plane
            // if the is Ax + By + Cz + D = 0, the distance of point x, y to this line is |Ax+By+Cz+D|/sqrt(A^2 + B^2 + C^2)
            float dist = fabs(x4 * A + y4 * B + C * z4 + D) / sqrt(A * A + B * B + C * C);

            // If distance is smaller than threshold count it as inlier
            if (dist <= distanceTol)
                iteration_inliers.insert(i);
        }
        //compare the iteration plane size with previous max plane
        if (iteration_inliers.size() > inliersResult.size())
            inliersResult = iteration_inliers;
    }

    // Return indicies of the points that were in the biggest plane found
    return inliersResult;
}

int main ()
{
	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

    //Load Data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	std::unordered_set<int> inliers = Ransac3D(cloud, 1000, 0.25);

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
