// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

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
#include<unordered_set>
#include<Eigen/Dense>
//#include "Cluster.h"
#include<random>

//#include "cluster.h"

//#include "/home/workspace/SFND_Lidar_Obstacle_Detection/src/KdTree.h"
// Structure to represent node of kd tree
/*---------Idea of using pcl::PointXYZI point is inspired from github.com/studian/SFND_P1_Lidar_Obstacle_Detection.git*/
struct Node
{
	//std::vector<typename PointT> point;
	pcl::PointXYZI point;
	int id;
	Node* left;
	Node* right;

	Node(/*std::vector<float> arr*/ pcl::PointXYZI arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

//template <typename PointT>
struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
	void inserthelper(Node** node, uint depth, pcl::PointXYZI point, int id)
	{
		if(*node == NULL)
		{
			*node = new Node(point, id);
		}
		else
        {
					int cd = depth % 3;
					//bool x = (point[cd] < ((*node)->point[cd]));
				
			
					if(cd==0)
					{
						if(point.x < ((*node)->point.x))
						{
							inserthelper((&(*node)-> left), depth+1, point, id);
						}
						else 
						{
							inserthelper((&(*node)-> right), depth+1, point, id);
						}
					}
					else 
					{
						if(point.y < ((*node)->point.y))
						{
							inserthelper((&(*node)-> left), depth+1, point, id);
						}
						else 
						{
							inserthelper((&(*node)-> right), depth+1, point, id);
						}
					}
		}	
	}
  
	void insert(pcl::PointXYZI point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		inserthelper(&root, 0,point, id);
		// the function should create a new node and place correctly with in the root 

	}

	void searchfunction(pcl::PointXYZI target,Node* node,int depth, float distanceTol, std::vector<int>& ids)
  {
		//int x=0;
		//int y=1;
		//int z=2;
		int cd=depth % 3;
		if (node!= NULL)
		{
			if ((node->point.x>= (target.x-distanceTol)&& node->point.x<=(target.x+distanceTol))&&(node->point.y>=(target.y-distanceTol)&& node->point.y<=(target.y+distanceTol))&&(node->point.z>=(target.z-distanceTol)&& node->point.z<=(target.z+distanceTol)))
			{
				/*Calculate the distance between the points */
				float caldistance = sqrtf((node->point.x-target.x)*(node->point.x-target.x)+(node->point.y-target.y)*(node->point.y-target.y)+(node->point.z-target.z)*(node->point.z-target.z));
				
				if(caldistance<=distanceTol)
				{
					ids.push_back(node->id);
				}
			}
			//check bounbdary conditions on both sides of the target node--target-distol and taget+distol
			if(cd==0)
			{
				if(target.x-distanceTol< node->point.x)
			{
				/* Assign the left side to contiue the search and incrase the depth for further accuracy*/
				searchfunction(target, node->left, depth+1, distanceTol, ids);
			}
			if(target.x+distanceTol> node->point.x)
			{
				/* Assign the right side to contiue the search and incrase the depth for further accuracy*/
				searchfunction(target, node->right, depth+1, distanceTol, ids);
			}
			}

			else
			{
				if(target.y-distanceTol< node->point.y)
			{
				/* Assign the left side to contiue the search and incrase the depth for further accuracy*/
				searchfunction(target, node->left, depth+1, distanceTol, ids);
			}
			if(target.y+distanceTol> node->point.y)
			{
				/* Assign the right side to contiue the search and incrase the depth for further accuracy*/
				searchfunction(target, node->right, depth+1, distanceTol, ids);
			}
			}
		}
		
  }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(pcl::PointXYZI target, float distanceTol)
	{
		std::vector<int> ids;
    searchfunction(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    
		void proximity(int index, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* Tree, float distanceTol);
		std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, int minSize, int maxSize);

		
		Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */