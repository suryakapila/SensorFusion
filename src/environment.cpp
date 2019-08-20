/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"


std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars,0);
  	pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud = lidar->scan();
    //renderRays(viewer, lidar -> position, inputcloud);
    //renderPointCloud(viewer, inputcloud, "inputcloud");
    
  // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(inputcloud, 100, 0.2);

  //  renderPointCloud(viewer, segmentCloud.first, "ObstCloud", Color(1,0,0) );
	//renderPointCloud(viewer, segmentCloud.second, "planeCloud",Color(0,1,0) );
   /*std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
       // if(render_clusters)
       // {
            std::cout << "cluster size ";
            pointProcessor->numPoints(cluster);
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
       // }
       
            Box box = pointProcessor->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        
        ++clusterId;
    } */
  
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloudI)
//Void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    //------------------------------
    //----open 3d viewer and display city block
    //---------------------------------
    
    
    bool render_clusters = true;
    bool render_box = true;
   

    //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    //pcl:: PointCloud<pcl::PointXYZI>::Ptr inputCloud= pointProcessorI->loadPcd("/home/workspace/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_1/0000000000.pcd");
    
    //render pointcloud from the data file
    //renderPointCloud(viewer, inputCloudI, "inputCloud");
    
    
    
    //experiment with filtered cloud for better picture and clear understanding
    float filterres = 0.2;
    Eigen::Vector4f minPoint (-8, -6,-2,1);
    Eigen::Vector4f maxPoint (30, 6,0.4,1);
    auto filtered_cloud = pointProcessorI->FilterCloud(inputCloudI, filterres, minPoint, maxPoint) ;
   
    //run RANSAC code in main()
    //set Max iterations and distance threshold for segmentation 
    int max_iterations = 250;
    float distanceTol  = 0.4;

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filtered_cloud, max_iterations, distanceTol);
    
     
	
    //renderPointCloud(viewer, filtered_cloud, "filtered_cloud");

    //Implementation of Clustering
    float clusterTol = 0.46;
    int minSize = 10;
    int maxSize = 600;

    //creating the kd tree for the search method of extraction
    KdTree* tree = new KdTree;
    
    for(int it =0;it<segmentCloud.first->points.size(); it++)
        tree->insert(segmentCloud.first->points[it], it);

   
    //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, clusterTol, minSize, maxSize);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->euclideanCluster(segmentCloud.first, tree, clusterTol, minSize, maxSize);


    renderPointCloud(viewer, segmentCloud.first, "ObstCloud", Color(1,0,0) );
    
    
    renderPointCloud(viewer, segmentCloud.second, "planeCloud",Color(0,1,0) );

    //render box around ego car
   // Box ego_box = {-1.5, -1.2,-1, 2.6,1.2, -0.4};
    //renderBox(viewer, ego_box, 0, Color(1,0,1), 0.6);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        if(render_clusters)
        {
            std::cout << "cluster size ";
            pointProcessorI->numPoints(cluster);
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        }
        if(render_box)
        {
            Box box = pointProcessorI->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        ++clusterId;
    } 
    
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
    //create point processor
    ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    //std::vector<boost::filesystem::path>stream = pointProcessor->streampcd("../src/sensors/data/pcd/data")
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud;
}


int main (int argc, char** argv)
{
    std::cout << "starting environment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
   
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    //std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_2");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    
    
    
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    //cityBlock(viewer);
    
    while (!viewer->wasStopped ())
    {
        //clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        //run Pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();
        viewer->spinOnce ();
    } 
}