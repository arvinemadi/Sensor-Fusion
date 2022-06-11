//Completed by Arvin.Emadi@Gmail.com 
// Example code and quiz from Udacity for a sensor fusion project
// Many leetcode tree examples are similar to this type of tree traversal
// The use of this is for quick search of neighbour pixels in a cloud of pixels
// Brute force approach is to go through all the points and check them one by one
// The approach here is based on Binary Search that reduces time complexity from O(n) to O(logn)
//Original owner of the code below
/* \author Aaron Brown */
// Quiz on implementing kd tree

#ifndef KDTREE_AR
#define KDTREE_AR

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

// Structure to represent nodes
template<typename T>
struct Node
{
	T point;
	int id;
	Node* left;
	Node* right;

	Node(T p, int setId)
	:	point(p), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

template<typename T>
struct KdTree
{
	Node<T>* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insert(T point, int id)
	{
		// Creating a binary traversal
		// Modification to normal binary traversal is that the decision depends on the depth
		// For two levels: if depth is even decision is based on x-value and if odd based on y-value of the point
		// For s levels: make decision based on depth % s - in this case s = 3
		BinaryTraverse(root, 0, point, id);
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(T target, float distanceTol)
	{
		std::vector<int> ids;
		BinarySearch(root, ids, target, 0, distanceTol);
		return ids;
	}

	void BinaryTraverse(Node<T>*& node, int depth, T point, int id)
	{
		if (node == nullptr)
			node = new Node<T>(point, id);
		else
		{
			int state_ = depth % 3;

			switch (state_)
			{
			case 0:
				
				if (point.x > (node->point).x)
					BinaryTraverse(node->right, depth + 1, point, id);
				else
					BinaryTraverse(node->left, depth + 1, point, id);
				break;

			case 1:
				if (point.y > (node->point).y)
					BinaryTraverse(node->right, depth + 1, point, id);
				else
					BinaryTraverse(node->left, depth + 1, point, id);
				break;

			case 2:

				if (point.z > (node->point).z)
					BinaryTraverse(node->right, depth + 1, point, id);
				else
					BinaryTraverse(node->left, depth + 1, point, id);
				break;
			}
			
		}
		return;
	}

	void BinarySearch(Node<T>*& node, std::vector<int>& ids, T& target, int depth, float& distanceTol)
	{
		int state_ = depth % 3;
		if (node == nullptr) return;
		if (fabs(target.x - (node->point).x) < distanceTol && fabs(target.y - (node->point).y && fabs(target.z - (node->point).z) < distanceTol))
		{
			if (sqrt((target.x - (node->point).x) * (target.x - (node->point).x) + (target.y - (node->point).y) * (target.y - (node->point).y) + (target.z - (node->point).z) * (target.z - (node->point).z)) <= distanceTol)
				ids.push_back(node->id);
			BinarySearch(node->left, ids, target, depth, distanceTol);
			BinarySearch(node->right, ids, target, depth, distanceTol);
		}
		else
		{
			switch (state_)
			{
			case 0:
				if (target.x - distanceTol < (node->point).x)
					BinarySearch(node->left, ids, target, depth + 1, distanceTol);
				if (target.x + distanceTol > (node->point).x)
					BinarySearch(node->right, ids, target, depth + 1, distanceTol);
				break;

			case 1:
				if (target.y - distanceTol < (node->point).y)
					BinarySearch(node->left, ids, target, depth + 1, distanceTol);
				if (target.y + distanceTol > (node->point).y)
					BinarySearch(node->right, ids, target, depth + 1, distanceTol);
				break;

			case 2:
				if (target.z - distanceTol < (node->point).z)
					BinarySearch(node->left, ids, target, depth + 1, distanceTol);
				if (target.z + distanceTol > (node->point).z)
					BinarySearch(node->right, ids, target, depth + 1, distanceTol);
				break;
			}
		}
	}

	//Depth First Search
	void FindProximity(const std::vector<T>& points, int point_id, std::vector<int>& cluster, std::vector<bool>& visited, float distanceTol, KdTree* tree)
	{
		visited[point_id] = true;
		cluster.push_back(point_id);
		std::vector<int> nearbyPoints = tree->search(points[point_id], distanceTol);

		for (int id : nearbyPoints)
		{
			if (!visited[id])
				FindProximity(points, id, cluster, visited, distanceTol, tree);
		}
		return;
	}



	std::vector<std::vector<int>> euclideanCluster(const std::vector<T>& points, KdTree* tree, float distanceTol)
	{
		// return list of indices for each cluster
		std::vector<std::vector<int>> clusters;
		int n = points.size();
		std::vector<bool> visited(n, false);

		for (int i = 0; i < n; i++)
		{
			if (!visited[i])
			{
				std::vector<int> cluster;
				FindProximity(points, i, cluster, visited, distanceTol, tree);
				cout << "Here is a cluster:" << endl;
				for (int pp : cluster)
					cout << pp << endl;
				clusters.push_back(cluster);
			}
		}
		return clusters;
	}
};


#endif

