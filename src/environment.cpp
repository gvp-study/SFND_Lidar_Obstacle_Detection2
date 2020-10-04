/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

// Global variables to test from the command line.
int maxIterations = 40;
float distanceThreshold = 0.3;
float clusterTolerance = 0.5;
int minsize = 10;
int maxsize = 140;


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
    //
    // Create lidar sensor
    //
    Lidar* lidar = new Lidar(cars, 0.0);
    //
    // Create point processor
    //
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcp = lidar->scan();
    //Vect3 origin = lidar->position;
    //renderRays(viewer, origin, pcp);
    //renderPointCloud(viewer, pcp, "Laser");
    // Process point cloud.
    ProcessPointClouds<pcl::PointXYZ> ppc;
    //
    // Segment plane
    //
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr>	
	segmentCloud = ppc.SegmentPlane(pcp, 100, 0.2);
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(1,1,1));
    //
    // Cluster different obstacle cloud
    //
    float clusterTolerance = 1.0;
    int minsize = 3;
    int maxsize = 30;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
	cloudClusters = ppc.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
    //
    // Find the bounding boxes for each cluster
    //
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size";
	ppc.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCLoud" + std::to_string(clusterId),
                         colors[clusterId % colors.size()]);
        // Find bounding boxes for each obstacle cluster
        Box box = ppc.BoundingBox(cluster);
        renderBox(viewer, box, clusterId, colors[clusterId % colors.size()], 0.5);
        ++clusterId;
    }
}

// Test load pcd
//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer){
//    ProcessPointClouds<pcl::PointXYZI>pointProcessor;
//    pcl::PointCloud<pcl::PointXYZI>::Ptr
//	inputCloud = pointProcessor.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
//    // renderPointCloud(viewer,inputCloud,"cloud");

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,
	       ProcessPointClouds<pcl::PointXYZI>* pointProcessor,
	       const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    //
    // Filter cloud to reduce amount of points
    //
    float filterRes = 0.4;
    Eigen::Vector4f minpoint(-10, -6.5, -2, 1);
    Eigen::Vector4f maxpoint(30, 6.5, 1, 1);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessor->FilterCloud(inputCloud,
										    filterRes,
										    minpoint,
										    maxpoint);
    //
    // Segment the filtered cloud into obstacles and road
    //
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr>
	segmentCloud = pointProcessor->SegmentPlane(filteredCloud, maxIterations, distanceThreshold);
    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(0, 0, 1));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
    //
    // Cluster the obstacle cloud
    //
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>
	cloudClusters = pointProcessor->Clustering(segmentCloud.first,
							     clusterTolerance,
							     minsize, maxsize);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 1, 0), Color(0, 1, 1), Color(1, 0, 1)};
    //
    // Find the bounding boxes for each cluster
    //
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId),
                         colors[clusterId % colors.size()]);
        // Find bounding boxes for each obstacle cluster
        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId, Color(1,0,0), 0.5);
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
}


int main (int argc, char** argv)
{

    if(argc > 1)
	maxIterations = atoi(argv[1]);
    if(argc > 2)
	distanceThreshold = atof(argv[2]);
    if(argc > 3)
	clusterTolerance = atof(argv[3]);
    if(argc > 4)
	minsize = atoi(argv[4]);
    if(argc > 5)
	maxsize = atoi(argv[5]);

    std::cout << "starting enviroment" << std::endl;
    std::cout << "maxIterations \t\t" << maxIterations << std::endl;
    std::cout << "distanceThreshold \t" << distanceThreshold << std::endl;
    std::cout << "clusterTolerance \t" << clusterTolerance << std::endl;
    std::cout << "minsize \t\t" << minsize << std::endl;
    std::cout << "maxsize \t\t" << maxsize << std::endl;
    

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
//    simpleHighway(viewer);
//    cityBlock(viewer);
//    while (!viewer->wasStopped ())
//    {
//        viewer->spinOnce ();
//    }
    //
    // Process a stream of lidar data and display the result.
    //
    ProcessPointClouds<pcl::PointXYZI>* pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path>
	stream = pointProcessor->streamPcd("../src/sensors/data/pcd/data_1");
//	stream = pointProcessor->streamPcd("../src/sensors/data/pcd/data_2");	
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;

    while (!viewer->wasStopped ())
    {
	// Clear viewer
	viewer->removeAllPointClouds();
	viewer->removeAllShapes();

	// Load pcd and run obstacle detection process
	inputCloud = pointProcessor->loadPcd((*streamIterator).string());
	cityBlock(viewer, pointProcessor, inputCloud);

	streamIterator++;
	if(streamIterator == stream.end())
	    streamIterator = stream.begin();

	viewer->spinOnce ();
    }
}
