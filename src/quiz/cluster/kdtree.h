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
	void inserthelper(Node** node, uint depth, std::vector<float> point, int id)
	{
		if(*node == NULL)
		{
			*node = new Node(point, id);
		}
		else
        {
					int cd = depth % 2;
					bool x = (point[cd] < ((*node)->point[cd]));
				
			
					if(x ==1)
					{
						inserthelper((&(*node)-> left), depth+1, point, id);
					}
					else 
					{
						inserthelper((&(*node)-> right), depth+1, point, id);
					}
		}	
	}
  
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		inserthelper(&root, 0,point, id);
		// the function should create a new node and place correctly with in the root 

	}

	void searchfunction(std::vector<float> target,Node* node,int depth, float distanceTol, std::vector<int>& ids)
  {
		int x=0;
		int y=1;
		int cd=depth % 2;
		if (node!= NULL)
		{
			if (((node->point[x]>= (target[x]-distanceTol)&& node->point[x]<=(target[x]+distanceTol)))&&(node->point[y]>=(target[y]-distanceTol)&& node->point[y]<=(target[y]+distanceTol)))
			{
				/*Calculate the distance between the points */
				float caldistance = sqrtf((node->point[x]-target[x])*(node->point[x]-target[x])+(node->point[y]-target[y])*(node->point[y]-target[y]));
				if(caldistance<=distanceTol)
				{
					ids.push_back(node->id);
				}
			}
			//check bounbdary conditions on both sides of the target node--target-distol and taget+distol
			if(target[cd]-distanceTol< node->point[cd])
			{
				/* Assign the left side to contiue the search and incrase the depth for further accuracy*/
				searchfunction(target, node->left, depth+1, distanceTol, ids);
			}
			if(target[cd]+distanceTol> node->point[cd])
			{
				/* Assign the right side to contiue the search and incrase the depth for further accuracy*/
				searchfunction(target, node->right, depth+1, distanceTol, ids);
			}
		}
		
  }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
    searchfunction(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};




