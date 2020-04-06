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
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		if(root==NULL){
			root=new Node(point,id);
			return;
		}
		Node* current=root;
		Node* parent=NULL;
		int d=0,cutdim=0;
		while(current!=NULL){
			parent=current;
			cutdim=d%2;
			if(point[cutdim]<current->point[cutdim]) current=current->left;
			else current=current->right;
			++d;
		}
		--d;
		cutdim=d%2;
		if(point[cutdim]<parent->point[cutdim]) parent->left=new Node(point,id);
		else parent->right=new Node(point,id);

	}

	void searchUtil(std::vector<float> target,float distanceTol,Node* node,int cutdim,std::vector<int>& ids){
		if(node!=NULL){
			if(target[0]-distanceTol<=node->point[0] && target[0]+distanceTol>=node->point[0] && target[1]-distanceTol<=node->point[1] && target[1]+distanceTol>=node->point[1]){
				float distance=sqrt((target[0] - node->point[0])*(target[0] - node->point[0]) + (target[1] - node->point[1])*(target[1] - node->point[1]));
				if(distance<=distanceTol) ids.push_back(node->id);
			}
			if(target[cutdim]-distanceTol<=node->point[cutdim]) searchUtil(target,distanceTol,node->left,(cutdim+1)%2,ids);
			if(target[cutdim]+distanceTol>=node->point[cutdim]) searchUtil(target,distanceTol,node->right,(cutdim+1)%2,ids);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchUtil(target,distanceTol,root,0,ids);
		return ids;
	}
	

};




