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

#include <unordered_set>
#include <unordered_map>

namespace kdtl{
    struct Node{
        std::vector<double> point;
        int id;
        Node* left;
        Node* right;

        Node(std::vector<double> _point,int _id):
        point(_point),id(_id),left(NULL),right(NULL){}

        ~Node(){
            if(left!=NULL) delete left;
            if(right!=NULL) delete right;
        }
    };

    template<int k>
    struct kdTree{
        Node* root;

        kdTree():
        root(NULL){}

        ~kdTree(){
            if(root!=NULL) delete root;
        }

        void insert(std::vector<double> point,int id){
            if(point.size()!=k) return;
            if(root==NULL){
                root=new Node(point,id);
                return;
            }
            Node* current=root;
            Node* parent=NULL;
            int d=0,cutdim=0;
            while(current!=NULL){
                parent=current;
                cutdim=d%k;
                if(point[cutdim]<current->point[cutdim]) current=current->left;
                else current=current->right;
                ++d;
            }
            --d;
            cutdim=d%k;
            if(point[cutdim]<parent->point[cutdim]) parent->left=new Node(point,id);
            else parent->right=new Node(point,id);
        }

        bool boundaryCheck(const std::vector<double>& target,const double distanceThreshold,Node* node){
            bool nodeInTargetBounds=true;
            for(int i=0;i<k;++i){
                nodeInTargetBounds=nodeInTargetBounds && (target[i]-distanceThreshold<=node->point[i] && target[i]+distanceThreshold>=node->point[i]);
            }
            return nodeInTargetBounds;
        }

        static double l2Distance(const std::vector<double>& point1, const std::vector<double>& point2){
            double distance_sq=0;
            for(int i=0;i<k;++i){
                distance_sq += (point1[i]-point2[i])*(point1[i]-point2[i]);
            }
            return sqrt(distance_sq);
        }

        void searchUtil(std::vector<double> target,double distanceThreshold,Node* node,int cutdim,std::vector<int>& ids){
            if(target.size()!=k) return;
            if(node!=NULL){
                if(boundaryCheck(target,distanceThreshold,node)){
                    double distance=l2Distance(target,node->point);
                    if(distance<=distanceThreshold) ids.push_back(node->id);
                }
                if(target[cutdim]-distanceThreshold<=node->point[cutdim]) searchUtil(target,distanceThreshold,node->left,(cutdim+1)%k,ids);
                if(target[cutdim]+distanceThreshold>=node->point[cutdim]) searchUtil(target,distanceThreshold,node->right,(cutdim+1)%k,ids);
            }
        }

        std::vector<int> search(std::vector<double> target,float distanceThreshold){
            std::vector<int> ids;
            searchUtil(target,distanceThreshold,root,0,ids);
            return ids;
        }
    };
}

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

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane_2(typename pcl::PointCloud<PointT>::Ptr cloud,int maxIterations,float distanceThreshold);
    void Clustering_2_util(typename pcl::PointCloud<PointT>::Ptr cloud,int id,kdtl::kdTree<3>* tree,double distanceThreshold,std::vector<int>& cluster,std::unordered_set<int>& processed);
    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering_2(typename pcl::PointCloud<PointT>::Ptr cloud, double distanceThreshold, int minSize, int maxSize);
    double volume(Box& box);
    double IOU_3D(Box& box1,Box& box2);
    void Tracking_2(std::map<int,Box>& BoxMap,std::map<int,int>& BoxTrackCount,std::vector<Box>& boxes_currentframe,bool trackinOn,int trackCountThresh);
    
};

#endif /* PROCESSPOINTCLOUDS_H_ */