//Completed by Arvin.Emadi@Gmail.com 
// Example code and quiz from Udacity for a sensor fusion project
//Original owner of the code below
/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
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

	

};




