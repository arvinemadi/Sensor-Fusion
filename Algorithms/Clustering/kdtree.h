//Completed by Arvin.Emadi@Gmail.com 
// Example code and quiz from Udacity for a sensor fusion project
// Many leetcode tree examples are similar to this type of tree traversal
// The use of this is for quick search of neighbour pixels in a cloud of pixels
// Brute force approach is to go through all the points and check them one by one
// The approach here is based on Binary Search that reduces time complexity from O(n) to O(logn)
//Original owner of the code below
/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent nodes
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insert(std::vector<float> point, int id)
	{
		// Creating a binary traversal
		// Modification to normal binary traversal is that the decision depends on the depth
		// if depth is even decision is based on x-value and if odd based on y-value of the point
		BinaryTraverse(root, 0, point, id);
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		BinarySearch(root, ids, target, 0, distanceTol);
		return ids;
	}
private:
	void BinaryTraverse(Node*& node, int depth, std::vector<float> point, int id)
	{
		if (node == nullptr)
			node = new Node(point, id);
		else
		{
			int state_ = depth % 2;
			if (point[state_] > node->point[state_])
				BinaryTraverse(node->right, depth + 1, point, id);
			else
				BinaryTraverse(node->left, depth + 1, point, id);
		}
		return;
	}

	void BinarySearch(Node*& node, std::vector<int>& ids, std::vector<float>& target, int depth, float& distanceTol)
	{
		int state_ = depth % 2;
		if (node == nullptr) return;
		if (fabs(target[0] - node->point[0]) < distanceTol && fabs(target[1] - node->point[1]) < distanceTol)
		{
			if (sqrt((target[0] - node->point[0]) * (target[0] - node->point[0]) + (target[1] - node->point[1]) * (target[1] - node->point[1])) <= distanceTol)
				ids.push_back(node->id);
			BinarySearch(node->left, ids, target, depth, distanceTol);
			BinarySearch(node->right, ids, target, depth, distanceTol);
		}
		else
		{
			if(target[state_] - distanceTol < node->point[state_])
				BinarySearch(node->left, ids, target, depth + 1, distanceTol);
			
			if (target[state_] + distanceTol > node->point[state_])
				BinarySearch(node->right, ids, target, depth + 1, distanceTol);

		}
	}
};




