#ifndef POLE_SEG_H
#define POLE_SEG_H

#include <ros/ros.h>
#include <chrono>
#include <sensor_msgs/PointCloud2.h>

//PCL
#include <pcl/PCLPointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>

// Transform listener
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include "tf/transform_datatypes.h"

//Service
#include <std_srvs/Empty.h>
#include <cstdlib>
#include <ros/service_traits.h>
#include "bachelor_thesis/PoleFound.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointXYZ;

class POLE_SEG //: public RECORD_PC
{
  using PointTPtr = typename pcl::PointCloud<PointT>::Ptr;

public:
    //Functions
    void Callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_pcd);
    void Callback_top(const sensor_msgs::PointCloud2::ConstPtr& cloud_pcd);
    void cloud_input_func();
    int cylinder_found_(const pcl::PointCloud<PointT>::Ptr &cloud_cylinder, pcl::ModelCoefficients::Ptr &coefficients_cylinder);
    void cylinder_can_be_pole_(const pcl::PointCloud<PointT>::Ptr &cloud_cylinder);

    void new_view_point_trajectory();
    bool Service(bachelor_thesis::PoleFound::Request& request, bachelor_thesis::PoleFound::Response& response);
    bool Service_real(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    void tf_transformer_bottom();
    void tf_transformer_pole();
    void tf_transformer_top();
    void tf_transformer_world_b();
    void tf_transformer_world_t();

    //Parameters
    bool bottom_not_in_view = 0;
    bool pole_in_edge_low = 0;    
    bool pole_in_edge_up = 0;
    bool top_not_in_view = 0;
    double drone_height_bottom;
    double drone_height_top;
    double duration_total = 0; //runtime of total algorithm
    double mean_plane_z = 0;
    double min_height_;
    double plane_max_z_ = 0; //max z-coordinate of plane
    double pole_on_floor_ = 0;
    Eigen::Vector3d position_vector_bottom;
    Eigen::Vector3d position_vector_top;
    Eigen::Vector4f plane_coeff_; //plane model parameters: a, b, c, d
    Eigen::Vector4d service_response; 
    Eigen::VectorXd pole_found_; //1 if pole found
    Eigen::VectorXf radius_ = Eigen::VectorXf::Zero(7); //radius of cylinders
    float length_cylinder_; //length of cylinder 
    int anzahl_cylinder_; //number of possibly correct cylinders found
    int current_cylinder_ = 0; //current number of cylinder 
    int cylinder_found_pole_ = 0; //depending on value cylinder found could be the pole or not

    //PCL point clouds PointT
    pcl::PointCloud<PointT>::Ptr all_cylinders_nothing_else_cloud {new pcl::PointCloud<PointT>};
    pcl::PointCloud<PointT>::Ptr cloud_ {new pcl::PointCloud<PointT>};
    pcl::PointCloud<PointT>::Ptr cloud_bodyframe_ {new pcl::PointCloud<PointT>};
    pcl::PointCloud<PointT>::Ptr cloud_bodyframe_b {new pcl::PointCloud<PointT>};
    pcl::PointCloud<PointT>::Ptr cloud_bodyframe_t {new pcl::PointCloud<PointT>};
    pcl::PointCloud<PointT>::Ptr cloud_filtered2 {new pcl::PointCloud<PointT>};
    pcl::PointCloud<PointT>::Ptr cloud_filtered_bot {new pcl::PointCloud<PointT>};
    pcl::PointCloud<PointT>::Ptr cloud_filtered_pass {new pcl::PointCloud<PointT>};
    pcl::PointCloud<PointT>::Ptr cloud_filtered_pass_top {new pcl::PointCloud<PointT>};    
    pcl::PointCloud<PointT>::Ptr cloud_filtered_top {new pcl::PointCloud<PointT>};
    pcl::PointCloud<PointT>::Ptr cloud_filtered_voxel {new pcl::PointCloud<PointT>};
    pcl::PointCloud<PointT>::Ptr cloud_input {new pcl::PointCloud<PointT>};
    pcl::PointCloud<PointT>::Ptr cloud_input_ {new pcl::PointCloud<PointT>};
    pcl::PointCloud<PointT>::Ptr cloud_plane {new pcl::PointCloud<PointT>};
    pcl::PointCloud<PointT>::Ptr cloud_pretransformed {new pcl::PointCloud<PointT>};
    pcl::PointCloud<PointT>::Ptr cloud_pretransformed_top {new pcl::PointCloud<PointT>};

    //PCL point clouds PointXYZ
    pcl::PointCloud<PointXYZ>::Ptr cloud_pretransformed_xyz {new pcl::PointCloud<PointXYZ>};
    pcl::PointCloud<PointXYZ>::Ptr cloud_pretransformed_xyz_t {new pcl::PointCloud<PointXYZ>};

    //sensor msg and converting to point cloud
    pcl::PCLPointCloud2 pcl_pc2;
    sensor_msgs::PointCloud2::ConstPtr recorded_pc_;      
    sensor_msgs::PointCloud2::ConstPtr recorded_pc_top_;      

    //PCL cloud normals
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals {new pcl::PointCloud<pcl::Normal>};
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 {new pcl::PointCloud<pcl::Normal>};

    //PCL filtering and extracting
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::ExtractIndices<PointT> extract;
    pcl::PassThrough<PointT> pass;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
    pcl::search::KdTree<PointT>::Ptr tree {new pcl::search::KdTree<PointT> ()}; 
    pcl::VoxelGrid<PointT> vgfilter;

    //Coefficients and inliers
    pcl::ModelCoefficients::Ptr coefficients_plane {new pcl::ModelCoefficients};
    pcl::PointIndices::Ptr inliers_cylinder_0 {new pcl::PointIndices};
    pcl::PointIndices::Ptr inliers_cylinder_1 {new pcl::PointIndices};
    pcl::PointIndices::Ptr inliers_cylinder_2 {new pcl::PointIndices};
    pcl::PointIndices::Ptr inliers_cylinder_3 {new pcl::PointIndices};
    pcl::PointIndices::Ptr inliers_cylinder_4 {new pcl::PointIndices};
    pcl::PointIndices::Ptr inliers_cylinder_5 {new pcl::PointIndices};
    pcl::PointIndices::Ptr inliers_cylinder_6 {new pcl::PointIndices}; 
    pcl::PointIndices::Ptr inliers_plane {new pcl::PointIndices};

    //write pdc files for visualising point cloud in rviz
    pcl::PCDWriter writer_;

    //PCL single data points
    pcl::PointXYZRGB CoM_pole_; //extreme point of current cylinder which is closer to plane

    //tf
    Eigen::Affine3d T_B_color_bot;
    Eigen::Affine3d T_B_color_top;
    Eigen::Affine3d T_B_pole_;
    Eigen::Affine3d T_B_world_b;
    Eigen::Affine3d T_B_world_t;
    tf::TransformListener tf_listener_;

    //Service
    int service_called = 0;
    ros::ServiceClient client_;
    ros::ServiceClient client2_;
};

#endif