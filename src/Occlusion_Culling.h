#ifndef OCCLUSION_H_
#define OCCLUSION_H_


#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "UAV.h"
 //PCL
#include <iostream>
#include <pcl/point_types.h>
#include "Frustum.h"
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>
#include "VoxelGridOcclusionEstimationT.h"

class OcclusionCulling
{
public:
    std::string model;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCopy;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr occlusionFreeCloud;//I can add it to accumulate cloud if I want to extract visible surface from multiple locations
    pcl::PointCloud<pcl::PointXYZ>::Ptr FrustumCloud;//frustum cull    
    pcl::PointCloud<pcl::PointXYZ> freeCloud;
    float voxelRes, OriginalVoxelsSize, viewEntropy;
    double id;
    pcl::VoxelGridOcclusionEstimationT voxelFilterOriginal;
    Eigen::Vector3i  max_b1, min_b1;

    //visualization_msgs::Marker linesList1, linesList2, linesList3, linesList4;
    //visualization_msgs::MarkerArray marker_array;
    pcl::FrustumCullingTT fc;
    double maxAccuracyError, minAccuracyError;
    bool AccuracyMaxSet;
    std::string frame_id;

    //methods
    OcclusionCulling(std::string modelName);
    OcclusionCulling(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudPtr);
    OcclusionCulling();
    ~OcclusionCulling();

    //chnage with position
    //pcl::PointCloud<pcl::PointXYZ> extractVisibleSurface(geometry_msgs::Pose location);
    pcl::PointCloud<pcl::PointXYZ> extractVisibleSurface(UAV camera);

    //    float calcCoveragePercent(geometry_msgs::Pose location);
    float calcCoveragePercent(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
    double calcAvgAccuracy(pcl::PointCloud<pcl::PointXYZ> pointCloud);
    //double calcAvgAccuracy(pcl::PointCloud<pcl::PointXYZ> pointCloud, geometry_msgs::Pose cameraPose);

    double calcAvgAccuracy(pcl::PointCloud<pcl::PointXYZ> pointCloud, UAV camera);

    //void transformPointMatVec(tf::Vector3 translation, tf::Matrix3x3 rotation, geometry_msgs::Point32 in, geometry_msgs::Point32& out);
    //pcl::PointCloud<pcl::PointXYZ> pointCloudViewportTransform(pcl::PointCloud<pcl::PointXYZ> pointCloud, geometry_msgs::Pose cameraPose);
    pcl::PointCloud<pcl::PointXYZ> pointCloudViewportTransform(pcl::PointCloud<pcl::PointXYZ> pointCloud, UAV camera);

    //void SSMaxMinAccuracy(std::vector<geometry_msgs::PoseArray> sensorsPoses);
    void SSMaxMinAccuracy(std::vector<UAV> uav_poses);

    //void visualizeFOV(geometry_msgs::Pose location);
    void visualizeFOV(UAV camera);
    //visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int id, int c_color[]);
    bool contains(pcl::PointCloud<pcl::PointXYZ> c, pcl::PointXYZ p);
    pcl::PointCloud<pcl::PointXYZ> pointsDifference(pcl::PointCloud<pcl::PointXYZ> c2);



    

};

#endif