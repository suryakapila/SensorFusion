/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include<chrono>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include<Eigen/Dense>
#include<iostream>

using namespace Eigen;

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
	auto startTime = std::chrono::steady_clock::now();
	// TODO: Fill in this function

	// For max iterations 
   /* while(maxIterations--)
	{
	// Randomly sample subset and fit line
		float x1,y1,x2,y2;
		//generate random values 
		std::unordered_set<int> inliers;
		
		while(inliers.size()<2)
			inliers.insert(rand()%cloud->points.size());
		auto itr = inliers.begin();
		x1= cloud->points[*itr].x;
		y1= cloud->points[*itr].y;
		itr++;
		x2= cloud->points[*itr].x;
		y2= cloud->points[*itr].y;
		//generate the line
		float a, b,c;
		a= y1-y2;
		b= x2-x1;
		c= ((x1*y2)-(x2*y1));
		for(int i=0; i<cloud->points.size();i++)
		{
			if(inliers.count(i)>0)
				continue;
			float x3,y3,d;
			pcl::PointXYZ point = cloud->points.at(i);
			x3= point.x;
			y3= point.y;
			d = fabs((a*x3+b*y3+c)/sqrt(a*a+b*b));
			if (d<=distanceTol)
				inliers.insert(i);
			
		}
	if (inliers.size() > inliersResult.size())
		inliersResult = inliers;
	} */ 
	

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	while(maxIterations--)
	{
		std::unordered_set<int> inliers;
		while(inliers.size()<3)
			inliers.insert(rand()%cloud->points.size());
		auto itr = inliers.begin();
		float x1 = cloud->points.at(*itr).x;
		float y1 = cloud->points.at(*itr).y;
		float z1 = cloud->points.at(*itr).z;
		itr++;
		float x2 = cloud->points.at(*itr).x;
		float y2 = cloud->points.at(*itr).y;
		float z2 = cloud->points.at(*itr).z;
		itr++;
		float x3 = cloud->points.at(*itr).x;
		float y3 = cloud->points.at(*itr).y;
		float z3 = cloud->points.at(*itr).z;

		Eigen::Vector3f v1(x2-x1, y2-y1, z2-z1);
		Eigen::Vector3f v2(x3-x1, y3-y1, z3-z1);
		Eigen::Vector3f v3 = v1.cross(v2);
		float i= v3.coeff(0);
		float j= v3.coeff(1);
		float k= v3.coeff(2);
		
		
		for(int index=0; index<cloud->points.size();index++)
		{
			if(inliers.count(index)>0)
				continue;
			
			pcl::PointXYZ point = cloud->points[index];
			float x4= point.x;
			float y4= point.y;
			float z4= point.z;

			float dis = fabs((i*(x4-x1)+ j*(y4-y1)+k*(z4-z1))/sqrt(i*i+j*j+k*k));
			if (dis <=distanceTol)
				inliers.insert(index);
			
		}
		if (inliers.size() > inliersResult.size())
			inliersResult = inliers;

	}
	
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "Ransac " << elapsedTime.count() << " milliseconds" << std::endl;
	
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.3);

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
