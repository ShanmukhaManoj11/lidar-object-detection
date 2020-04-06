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
    std::shared_ptr<Lidar> lidar=std::make_shared<Lidar>(cars,0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud=lidar->scan();
    //renderRays(viewer,lidar->position,pointcloud);
    //renderPointCloud(viewer,pointcloud,"PointCloud",Color(1,1,1));

    // TODO:: Create point processor
    std::shared_ptr< ProcessPointClouds<pcl::PointXYZ> > processor=std::make_shared< ProcessPointClouds<pcl::PointXYZ> >();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud=processor->SegmentPlane(pointcloud,100,0.2);
    //renderPointCloud(viewer,segmentCloud.first,"planeCloud",Color(0,1,0));
    //renderPointCloud(viewer,segmentCloud.second,"obstCloud",Color(1,0,0));
    
    std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters=processor->Clustering(segmentCloud.second,1.0,3,30);
    int clusterId=0;
    std::vector<Color> colors={Color(1,0,0),Color(0,1,0),Color(0,0,1)};
    for(auto cluster: clusters){
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%3]);
        Box box=processor->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
  
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,const std::shared_ptr< ProcessPointClouds<pcl::PointXYZI> >& processor,const pcl::PointCloud<pcl::PointXYZI>::Ptr& pointcloud,std::map<int,Box>& BoxMap,std::map<int,int>& BoxTrackCount){
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud=processor->FilterCloud(pointcloud,0.2,Eigen::Vector4f(-10,-6,-2,1),Eigen::Vector4f(30,6,1,1));
    //renderPointCloud(viewer,filteredCloud,"filteredCloud");

    //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud=processor->SegmentPlane(filteredCloud,50,0.2);
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud=processor->SegmentPlane_2(filteredCloud,50,0.2);
    renderPointCloud(viewer,segmentCloud.first,"planeCloud",Color(0,1,0));
    //renderPointCloud(viewer,segmentCloud.second,"obstCloud",Color(1,0,0));

    //std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr > clusters=processor->Clustering(segmentCloud.second,0.5,10,5000); 
    std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr > clusters=processor->Clustering_2(segmentCloud.second,0.5,10,5000);
    
    int clusterId=0;
    std::vector<Color> colors={Color(1,0,0),Color(1,1,0),Color(0,0,1)};
    std::vector<Box> boxes;
    for(auto cluster: clusters){
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%3]);
        Box box=processor->BoundingBox(cluster);
        boxes.push_back(box);
        ++clusterId;
    }   

    int trackCountThresh=1;
    processor->Tracking_2(BoxMap,BoxTrackCount,boxes,true,trackCountThresh);

    for(auto boxmap_it=BoxMap.begin();boxmap_it!=BoxMap.end();++boxmap_it){
        if(BoxTrackCount[boxmap_it->first]>=trackCountThresh){
            renderBox(viewer,boxmap_it->second,boxmap_it->first);
        }
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
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    
    std::shared_ptr< ProcessPointClouds<pcl::PointXYZI> > processor=std::make_shared< ProcessPointClouds<pcl::PointXYZI> >();
    std::vector<boost::filesystem::path> stream=processor->streamPcd("../src/sensors/data/pcd/data_1");
    auto stream_it=stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud;
    std::map<int,Box> BoxMap;
    std::map<int,int> BoxTrackCount;

    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        pointcloud=processor->loadPcd(stream_it->string());
        cityBlock(viewer,processor,pointcloud,BoxMap,BoxTrackCount);

        ++stream_it;
        if(stream_it==stream.end()) stream_it=stream.begin();

        viewer->spinOnce ();
    } 
}