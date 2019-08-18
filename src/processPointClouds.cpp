// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
//#include "cluster.cpp"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    //create a filtering object: downsample the dataset using the leafsize of 0.2m
    pcl::VoxelGrid<PointT> sor;
    typename pcl::PointCloud<PointT>::Ptr filtercloud (new pcl::PointCloud<PointT>);

    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes,filterRes,filterRes);
    sor.filter(*filtercloud);

    //create an area of interest
    typename pcl::PointCloud<PointT>::Ptr cloudregion(new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(filtercloud);
    region.filter(*cloudregion);

    //optional: crop the roof top points of ego car
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.8, -1.9,-1.1,1));
    roof.setMax(Eigen::Vector4f(2.7, 1.9, -0.2,1));
    roof. setInputCloud(filtercloud);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    for(int index:indices)
    {
        inliers->indices.push_back(index);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudregion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudregion);



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudregion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  	typename pcl::PointCloud<PointT>::Ptr ObstCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr PlaneCloud (new pcl::PointCloud<PointT> ());
    for(int index : inliers->indices)
        PlaneCloud->points.push_back(cloud->points[index]);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
	extract.filter(*ObstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(ObstCloud, PlaneCloud);
    return segResult;
}




template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
  
    std::unordered_set<int> inlierResult;
    
	srand(time(NULL));
	auto startTime = std::chrono::steady_clock::now();
	// TODO: Fill in this function

	// For max iterations 
   	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	while(maxIterations--)
	{
		std::unordered_set<int> in;
		
        while(in.size()<3)
			in.insert(rand()%cloud->points.size());
		auto itr = in.begin();
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
		
		
		for(size_t index=0; index<cloud->points.size();index++)
		{
			if(in.count(index)>0)
				continue;
			
		    //auto point = cloud->points[index];
			float x4= cloud->points[index].x;
			float y4= cloud->points[index].y;
			float z4= cloud->points[index].z;

			float dis = fabs((i*(x4-x1)+ j*(y4-y1)+k*(z4-z1))/sqrt(i*i+j*j+k*k));
			if (dis <=distanceThreshold)
			{
                in.insert(index);
                //inliers->indices.push_back(index);
            }	
			
		}
        if(in.size()>inlierResult.size())
            inlierResult = in;
    }	
    //transfer all the points in unordered_set into pointindices inliers
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int i : inlierResult)
    {
        inliers->indices.push_back(i);
    }
    
	
    // TODO:: Fill in this function to find inliers for the cloud using PCL library
/*pcl::SACSegmentation<PointT> seg;       //creating a Seg object
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    //Segment the largest planar component form the input cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);*/
  
    if(inliers -> indices.size() == 0)
    {
       std::cerr<<"could not estimate a planar segment for the given dataset"<<std::endl;
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}



 
template<typename PointT>

std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    //creating the kd tree for the search method of extraction
    //KdTree* tree = new KdTree;
    /*std::vector<std::vector<float>> points;
    for(int i=0; i<cloud->points.size(); i++)
    {
        auto p = cloud->points[i];
        std::vector<float> point = {p.x, p.y,p.z};
        points.push_back(point);
        tree->insert(points[i], i);
    }*/
    
  //Implemented Eucleadian clustering using PCL library
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMaxClusterSize(maxSize);
    ec.setMinClusterSize(minSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    
   // std::vector<std::vector<int>> cluster_indices = euclideanCluster(points, tree, clusterTolerance);
    
    //for(std vector<pcl::PointIndices> const_iterator it = cluster_indices.begin(); it!= cluster_indices.end(); it++ )
   for (pcl::PointIndices getIndices: cluster_indices)
    //for(std::vector<int> getIndices: cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudClusters(new pcl::PointCloud<PointT>);
        //for(std::vector<int> pit = it->indices.begin(); pit !=it->indices.end(); ++pit)
        for(int index: getIndices.indices)
            cloudClusters->points.push_back(cloud->points.at(index));
        cloudClusters->width = cloudClusters->points.size();
        cloudClusters->height = 1;
        cloudClusters->is_dense = true;
        
        clusters.push_back(cloudClusters);
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

  


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}