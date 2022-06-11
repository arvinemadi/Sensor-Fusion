//Original author of base code is referenced below - This is part of a Udacity sensor fusion project
// this 3D RANSAC algithm is implemented by Arvin.Emadi@Gmail.com

/* \author Aaron Brown */
// using templates for processPointClouds so also include .cpp to help linker

#ifndef RANSAC3D_AR
#define RANSAC3D_AR

#include <unordered_set>

// PCL lib Functions for processing point clouds 

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include "boost/filesystem.hpp"
#include <iostream>

#include <boost/type_traits/conjunction.hpp>
#include <boost/type_traits/disjunction.hpp>
//Main function of the RANSAC algorithm
template<typename T>
typename std::unordered_set<int> My_Ransac3D(typename T cloud, int maxIterations, float distanceTol)
{
    //Some control for debug
    bool show_comments = false;
    int n_points = cloud->points.size();
    std::unordered_set<int> inliersResult;              // the biggest set will be always stored here during iterations
    auto startTime = std::chrono::steady_clock::now();  // keeping track of time
                          

    // For # iterations of iterations pick 3 points randomly - make a plane with them - add any other point that is within distance tolerance to the plane
    // At the end of each iteration check if the current plane is bigger that previous max plane and if yes replace that with current plane
    while (maxIterations > 0) {
        maxIterations--;
        //std::cout << "iteration number is " << maxIterations << endl;
        
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
        if (iteration_inliers.size() > inliersResult.size()) {
            inliersResult = iteration_inliers;
            if ((float)iteration_inliers.size() / (float)n_points > 0.75)
                break;
        }
        if (show_comments)
        {
            auto endTime = std::chrono::steady_clock::now();
            auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
            std::cout << "This iteration took " << elapsedTime.count() << " milliseconds and the plane size is : " << (float)iteration_inliers.size() / (float)n_points << std::endl;
            startTime = endTime;
        }
    }

    // Return indicies of the points that were in the biggest plane found
    return inliersResult;
}

#endif