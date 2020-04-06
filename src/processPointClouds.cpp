// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

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
    typename pcl::PointCloud<PointT>::Ptr filteredCloud(new typename pcl::PointCloud<PointT>());

    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes,filterRes,filterRes);
    vg.filter(*filteredCloud);

    pcl::CropBox<PointT> cb;
    cb.setInputCloud(filteredCloud);
    cb.setMin(minPoint);
    cb.setMax(maxPoint);
    cb.filter(*filteredCloud);

    //remove roof points
    std::vector<int> roof_indices;
    pcl::CropBox<PointT> cb_roof;
    cb_roof.setInputCloud(filteredCloud);
    cb_roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    cb_roof.setMax(Eigen::Vector4f(2.6,1.7,-0.4,1));
    cb_roof.filter(roof_indices);
    pcl::PointIndices::Ptr cb_roof_inliers(new pcl::PointIndices());
    for(int id: roof_indices){
        cb_roof_inliers->indices.push_back(id);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(filteredCloud);
    extract.setIndices(cb_roof_inliers);
    extract.setNegative(true);
    extract.filter(*filteredCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filteredCloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    typename pcl::PointCloud<PointT>::Ptr cloud_p(new typename pcl::PointCloud<PointT>()); //points corresponding to plane
    extract.setNegative(false);
    extract.filter(*cloud_p);
    typename pcl::PointCloud<PointT>::Ptr cloud_np(new typename pcl::PointCloud<PointT>()); //points not on plane
    extract.setNegative(true);
    extract.filter(*cloud_np);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_p, cloud_np);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers,*coefficients);
    if(inliers->indices.size()==0){
        std::cout<<"Could not estimate planar model for the given dataset"<<std::endl;
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
    typename pcl::search::KdTree<PointT>::Ptr kdtree(new typename pcl::search::KdTree<PointT>());
    kdtree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    typename pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(kdtree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for(auto it=cluster_indices.begin();it!=cluster_indices.end();++it){
    	typename pcl::PointCloud<PointT>::Ptr cluster(new typename pcl::PointCloud<PointT>());
    	for(auto pit=it->indices.begin();pit!=it->indices.end();++pit){
    		cluster->points.push_back(cloud->points[*pit]);
    	}
    	cluster->width=cloud->points.size();
    	cluster->height=1;
    	cluster->is_dense=true;
    	clusters.push_back(cluster);
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

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane_2(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold){
    auto startTime=std::chrono::steady_clock::now();

    std::unordered_set<int> inliers;
    srand(time(NULL));

    int npoints=cloud->points.size();
    for(int iter=0;iter<maxIterations;++iter){
        std::unordered_set<int> iteration_inliers;
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
                iteration_inliers.insert(i);
            }
            else{
                double x=cloud->points[i].x, y=cloud->points[i].y, z=cloud->points[i].z;
                double dist=abs(a*x+b*y+c*z+d)/sqrt(a*a+b*b+c*c);
                if(dist<=distanceThreshold){
                    iteration_inliers.insert(i);
                }
            }
        }
        if(inliers.size()<iteration_inliers.size()){
            inliers=iteration_inliers;
        }
    }

    typename pcl::PointCloud<PointT>::Ptr planeCloud(new typename pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new typename pcl::PointCloud<PointT>());
    for(int i=0;i<npoints;++i){
        PointT point=cloud->points[i];
        if(inliers.find(i)==inliers.end()) obstCloud->points.push_back(point);
        else planeCloud->points.push_back(point);
    }

    auto endTime=std::chrono::steady_clock::now();
    auto elapsedTime=std::chrono::duration_cast<std::chrono::milliseconds>(endTime-startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return std::make_pair(planeCloud,obstCloud);
}

template<typename PointT>
void ProcessPointClouds<PointT>::Clustering_2_util(typename pcl::PointCloud<PointT>::Ptr cloud,int id,kdtl::kdTree<3>* tree,double distanceThreshold,std::vector<int>& cluster,std::unordered_set<int>& processed){
    if(processed.find(id)==processed.end()){
        processed.insert(id);
        cluster.push_back(id);
        std::vector<double> point{cloud->points[id].x,cloud->points[id].y,cloud->points[id].z};
        std::vector<int> neighbor_ids=tree->search(point,distanceThreshold);
        for(int neighbor_id: neighbor_ids){
            if(processed.find(neighbor_id)==processed.end()) Clustering_2_util(cloud,neighbor_id,tree,distanceThreshold,cluster,processed);
        }
    }
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering_2(typename pcl::PointCloud<PointT>::Ptr cloud, double distanceThreshold, int minSize, int maxSize){
    auto startTime=std::chrono::steady_clock::now();

    kdtl::kdTree<3>* tree=new kdtl::kdTree<3>();
    for(int id=0;id<cloud->points.size();++id){
        tree->insert(std::vector<double>{cloud->points[id].x,cloud->points[id].y,cloud->points[id].z},id);
    }

    std::vector< typename pcl::PointCloud<PointT>::Ptr > clusterClouds;
    std::unordered_set<int> processed;
    for(int id=0;id<cloud->points.size();++id){
        if(processed.find(id)==processed.end()){
            std::vector<int> cluster;
            Clustering_2_util(cloud,id,tree,distanceThreshold,cluster,processed);

            if(cluster.size()>=minSize && cluster.size()<=maxSize){
                typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
                for(int pid: cluster){
                    clusterCloud->points.push_back(cloud->points[pid]);
                }
                clusterClouds.push_back(clusterCloud);
            }
        }
    }

    auto endTime=std::chrono::steady_clock::now();
    auto elapsedTime=std::chrono::duration_cast<std::chrono::milliseconds>(endTime-startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds" << std::endl;    

    delete tree;
    return clusterClouds;
}

template<typename PointT>
double ProcessPointClouds<PointT>::volume(Box& box){
    return std::max(0.0,double(box.x_max-box.x_min))*std::max(0.0,double(box.y_max-box.y_min))*std::max(0.0,double(box.z_max-box.z_min));
}

template<typename PointT>
double ProcessPointClouds<PointT>::IOU_3D(Box& box1,Box& box2){
    Box box;
    box.x_min=std::max(box1.x_min,box2.x_min);
    box.y_min=std::max(box1.y_min,box2.y_min);
    box.z_min=std::max(box1.z_min,box2.z_min);
    box.x_max=std::min(box1.x_max,box2.x_max);
    box.y_max=std::min(box1.y_max,box2.y_max);
    box.z_max=std::min(box1.z_max,box2.z_max);

    double intersection_volume=volume(box);
    double union_volume=volume(box1)+volume(box2)-intersection_volume;
    return intersection_volume/union_volume;
}

template<typename PointT>
void ProcessPointClouds<PointT>::Tracking_2(std::map<int,Box>& BoxMap,std::map<int,int>& BoxTrackCount,std::vector<Box>& boxes_currentframe,bool trackingOn,int trackCountThresh){
    if(!trackingOn){
        BoxMap.clear();
        BoxTrackCount.clear();
        int boxId=0;
        for(auto box: boxes_currentframe){
            BoxMap[boxId]=box;
            BoxTrackCount[boxId]=trackCountThresh;
            ++boxId;
        }
    }
    else{
        auto startTime=std::chrono::steady_clock::now();
        if(BoxMap.empty()){
            int boxId=0;
            for(auto box: boxes_currentframe){
                BoxMap[boxId]=box;
                BoxTrackCount[boxId]=1;
                ++boxId;
            }
        }
        else{
            double filterCoeff_x=0.25,filterCoeff_y=0.25,filterCoeff_z=0.25;
            std::unordered_set<int> detectedBoxIds;
            for(int i=0;i<boxes_currentframe.size();++i){
                double max_iou=0;
                int similar_boxId=-1;
                for(auto boxmap_it=BoxMap.begin();boxmap_it!=BoxMap.end();++boxmap_it){
                    double iou=IOU_3D(boxes_currentframe[i],boxmap_it->second);
                    if(iou>max_iou){
                        max_iou=iou;
                        similar_boxId=boxmap_it->first;
                    }
                }
                if(similar_boxId==-1){
                    int boxId=0;
                    auto boxmap_it=BoxMap.begin();
                    while(boxmap_it!=BoxMap.end() && boxmap_it->first<=boxId){
                        ++boxmap_it;
                        ++boxId;
                    }
                    BoxMap[boxId]=boxes_currentframe[i];
                    BoxTrackCount[boxId]=1;
                    detectedBoxIds.insert(boxId);
                }
                else{
                    Box box_cf=boxes_currentframe[i];
                    std::vector<double> box_cf_dim{box_cf.x_max-box_cf.x_min,box_cf.y_max-box_cf.y_min,box_cf.z_max-box_cf.z_min};
                    std::vector<double> box_cf_center{0.5*(box_cf.x_min+box_cf.x_max),0.5*(box_cf.y_min+box_cf.y_max),0.5*(box_cf.z_min+box_cf.z_max)};
                    Box box_pf=BoxMap[similar_boxId];
                    std::vector<double> box_pf_dim{box_pf.x_max-box_pf.x_min,box_pf.y_max-box_pf.y_min,box_pf.z_max-box_pf.z_min};
                    
                    std::vector<double> box_filtered_dim{filterCoeff_x*box_cf_dim[0]+(1-filterCoeff_x)*box_pf_dim[0],filterCoeff_y*box_cf_dim[1]+(1-filterCoeff_y)*box_pf_dim[1],filterCoeff_z*box_cf_dim[2]+(1-filterCoeff_z)*box_pf_dim[2]};
                    Box box_filtered;
                    box_filtered.x_min=box_cf_center[0]-0.5*box_filtered_dim[0];
                    box_filtered.y_min=box_cf_center[1]-0.5*box_filtered_dim[1];
                    box_filtered.z_min=box_cf_center[2]-0.5*box_filtered_dim[2];
                    box_filtered.x_max=box_cf_center[0]+0.5*box_filtered_dim[0];
                    box_filtered.y_max=box_cf_center[1]+0.5*box_filtered_dim[1];
                    box_filtered.z_max=box_cf_center[2]+0.5*box_filtered_dim[2];

                    BoxMap[similar_boxId]=box_filtered;
                    if(BoxTrackCount[similar_boxId]<trackCountThresh) ++BoxTrackCount[similar_boxId];
                    detectedBoxIds.insert(similar_boxId);
                }
            }
            for(auto boxmap_it=BoxMap.begin();boxmap_it!=BoxMap.end();++boxmap_it){
                if(detectedBoxIds.find(boxmap_it->first)==detectedBoxIds.end()){
                    BoxMap.erase(boxmap_it);
                }
            }
            for(auto boxtrackcount_it=BoxTrackCount.begin();boxtrackcount_it!=BoxTrackCount.end();++boxtrackcount_it){
                if(detectedBoxIds.find(boxtrackcount_it->first)==detectedBoxIds.end()){
                    BoxTrackCount.erase(boxtrackcount_it);
                }
            }
        }
        auto endTime=std::chrono::steady_clock::now();
        auto elapsedTime=std::chrono::duration_cast<std::chrono::milliseconds>(endTime-startTime);
        std::cout << "tracking took " << elapsedTime.count() << " milliseconds" << std::endl;    
    }
}