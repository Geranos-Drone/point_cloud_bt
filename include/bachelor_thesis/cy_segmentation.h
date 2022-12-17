#ifndef POLE_SEG_H
#define POLE_SEG_H

#include <ros/ros.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/features/normal_3d.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>
#include <pcl/filters/extract_indices.h>

// Transform listener
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include "tf/transform_datatypes.h"

//Service
#include <std_srvs/Empty.h>
#include <cstdlib>
/*
#include <sensor_msgs/NavSatFix.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/transforms.h>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>
#include "std_msgs/String.h"
#include <std_msgs/Float32.h>
#include <string>
#include "gtest/gtest.h"
#include "Eigen/Core"
#include "Eigen/Geometry" */

typedef pcl::PointXYZRGB PointT;


class POLE_SEG
{
  using PointTPtr = typename pcl::PointCloud<PointT>::Ptr;

public:
    pcl::PCDWriter writer_;
    int cylinder_number_ = 0; //current number of cylinder 
    Eigen::Vector4f plane_coeff_;
    int anzahl_cylinder_; //number of possibly correct cylinders found
    bool callback_called_once_ = 0;

    pcl::PCLPointCloud2 pcl_pc2;
    pcl::PointCloud<PointT>::Ptr cloud {new pcl::PointCloud<PointT>};
    pcl::PointCloud<PointT>::Ptr cloud_filtered {new pcl::PointCloud<PointT>};
    //Filter
    pcl::VoxelGrid<PointT> vgfilter;
    pcl::PassThrough<PointT> pass;
    pcl::PointCloud<PointT>::Ptr cloud_filtered_voxel {new pcl::PointCloud<PointT>};
    pcl::search::KdTree<PointT>::Ptr tree {new pcl::search::KdTree<PointT> ()}; 
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals {new pcl::PointCloud<pcl::Normal>};
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
    pcl::ExtractIndices<PointT> extract;
    pcl::PointCloud<PointT>::Ptr cloud_plane {new pcl::PointCloud<PointT>};
    pcl::PointCloud<PointT>::Ptr cloud_filtered2 {new pcl::PointCloud<PointT>};
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 {new pcl::PointCloud<pcl::Normal>};

    //pcl::PointIndices::Ptr inliers_plane {new pcl::PointIndices};
    pcl::PointIndices::Ptr inliers_cylinder_0 {new pcl::PointIndices};
    pcl::PointIndices::Ptr inliers_cylinder_1 {new pcl::PointIndices};
    pcl::PointIndices::Ptr inliers_cylinder_2 {new pcl::PointIndices};
    pcl::PointIndices::Ptr inliers_cylinder_3 {new pcl::PointIndices};
    pcl::PointIndices::Ptr inliers_cylinder_4 {new pcl::PointIndices};
    pcl::PointIndices::Ptr inliers_cylinder_5 {new pcl::PointIndices};
    pcl::PointIndices::Ptr inliers_cylinder_6 {new pcl::PointIndices}; 



    pcl::PointIndices::Ptr inliers_plane {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients_plane {new pcl::ModelCoefficients};


    //needed to see if previous found cylinders similar 
    pcl::PointXYZRGB point_minmax_; //extreme point of current cylinder which is closer to plane
    pcl::PointXYZRGB point_minmax_0_; //extreme point of 0th cylinder which is closer to plane
    pcl::PointXYZRGB point_minmax_1_; //extreme point of 1st cylinder which is closer to plane
    pcl::PointXYZRGB point_minmax_2_; //extreme point of 2nd cylinder which is closer to plane
    pcl::PointXYZRGB point_minmax_3_; //extreme point of 3rd cylinder which is closer to plane
    pcl::PointXYZRGB point_minmax_4_; //extreme point of 4th cylinder which is closer to plane
    pcl::PointXYZRGB point_minmax_5_; //extreme point of 5th cylinder which is closer to planed

    int current_cylinder_; //current number of cylinder 
    pcl::PointXYZRGB point_ontop_; //extreme point of current cylinder which is closer to plane
    float radius_kdtree_; //radius used for kdtree (freestanding pole criteria)
    float length_axis_real_; //projected length
    int cylinder_found_pole_ = 0; //depending on value cylinder found could be the pole or not
    int pointminmax_;
    Eigen::VectorXf radius_ = Eigen::VectorXf::Zero(7); //radius of cylinders
    Eigen::VectorXf cylinder_red_or_green_ = Eigen::VectorXf::Zero(7); //0 if red and 1 if green

    //tf
    tf::TransformListener tf_listener_;
    Eigen::Affine3d T_B_color_;

    //functions
    void callback_(const sensor_msgs::PointCloud2::ConstPtr& cloud_pcd);
    void segmentation_(const sensor_msgs::PointCloud2::ConstPtr& cloud_pcd);
    void tf_transformer(const pcl::PointXYZRGB& point_on_pole);
    int cylinder_found_(const pcl::PointCloud<PointT>::Ptr &cloud_cylinder, pcl::ModelCoefficients::Ptr &coefficients_cylinder);
    void cylinder_can_be_pole_(const pcl::PointCloud<PointT>::Ptr &cloud_cylinder);
    void color_only_cylinders_(const pcl::PointCloud<PointT>::Ptr &cloud_0,const pcl::PointCloud<PointT>::Ptr &cloud_1,const pcl::PointCloud<PointT>::Ptr &cloud_2,const pcl::PointCloud<PointT>::Ptr &cloud_3,const pcl::PointCloud<PointT>::Ptr &cloud_4,const pcl::PointCloud<PointT>::Ptr &cloud_5,const pcl::PointCloud<PointT>::Ptr &cloud_6);
  
    //GUI Service
    ros::NodeHandle nh_;
    ros::ServiceClient pole_detection_;

};

#endif