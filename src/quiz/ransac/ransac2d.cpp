/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

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
	auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
  int npoints=cloud->points.size();
  for(int iter=0;iter<maxIterations;++iter){
    std::unordered_set<int> inliers;
    int i1=0, i2=0;
    while(i1==i2){
      i1=rand()%npoints;
      i2=rand()%npoints;
    }
    double a=cloud->points[i1].y-cloud->points[i2].y;
    double b=cloud->points[i2].x-cloud->points[i1].x;
    double c=(cloud->points[i1].x * cloud->points[i2].y)-(cloud->points[i2].x * cloud->points[i1].y);
    for(int i=0;i<npoints;++i){
      double x=cloud->points[i].x, y=cloud->points[i].y;
      double dist=abs(a*x+b*y+c)/sqrt(a*a+b*b);
      if(dist<=distanceTol){
        inliers.insert(i);
      }
    }
    if(inliers.size()>inliersResult.size()){
      inliersResult=inliers;
    }
  }

  auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "RANSAC line fit took " << elapsedTime.count() << " milliseconds" << std::endl;
	
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
  int npoints=cloud->points.size();
  for(int iter=0;iter<maxIterations;++iter){
    std::unordered_set<int> inliers;
    int i1=rand()%npoints;
    int i2=rand()%npoints;
    while(i1==i2){
    	i2=rand()%npoints;
    }
    int i3=rand()%npoints;
    while(i1==i3 || i2==i3){
    	i3=rand()%npoints;
    }
    double x1=cloud->points[i1].x, y1=cloud->points[i1].y, z1=cloud->points[i1].z;
    double x2=cloud->points[i2].x, y2=cloud->points[i2].y, z2=cloud->points[i2].z;
    double x3=cloud->points[i3].x, y3=cloud->points[i3].y, z3=cloud->points[i3].z;
    double a=(y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
    double b=(z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
    double c=(x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
    double d=-(a*x1+b*y1+c*z1);
    for(int i=0;i<npoints;++i){
    	if(i==i1 || i==i2 || i==i3){
    		inliers.insert(i);
    	}
    	else{
    		double x=cloud->points[i].x, y=cloud->points[i].y, z=cloud->points[i].z;
			double dist=abs(a*x+b*y+c*z+d)/sqrt(a*a+b*b+c*c);
			if(dist<=distanceTol){
				inliers.insert(i);
			}
    	}
    }
    if(inliers.size()>inliersResult.size()){
      inliersResult=inliers;
    }
  }

  auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "RANSAC plane fit took " << elapsedTime.count() << " milliseconds" << std::endl;
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 50, 1.0);
	std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.2);

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
