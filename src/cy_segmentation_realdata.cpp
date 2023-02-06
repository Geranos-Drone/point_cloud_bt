#include <bachelor_thesis/cy_segmentation.h>
#include <math.h> 
#include <algorithm>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std::chrono;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointXYZ;


void POLE_SEG::Callback (const sensor_msgs::PointCloud2::ConstPtr& cloud_pcd) {                             
  ROS_INFO_ONCE("Record_PC received first Pointcloud bottom!");
  recorded_pc_ = cloud_pcd;
  tf_transformer_world_b();
}

void POLE_SEG::Callback_top (const sensor_msgs::PointCloud2::ConstPtr& cloud_pcd) {                             
  ROS_INFO_ONCE("Record_PC received first Pointcloud top!");
  recorded_pc_top_ = cloud_pcd;
  tf_transformer_world_t();
}

bool POLE_SEG::Service_real(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  auto start = high_resolution_clock::now();
  
  ++service_called;
  pole_found_.resize(7);
  pole_found_ << 0,0,0,0,0,0,0;
  std::cerr << "service called: " << service_called << std::endl;

  //converts sensor_msgs to pointcloud
  pcl_conversions::toPCL(*recorded_pc_,pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2,*cloud_pretransformed_xyz);
  pcl_conversions::toPCL(*recorded_pc_top_,pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2,*cloud_pretransformed_xyz_t);
  
  //from PointXYZ to PointXYZRGB
  cloud_pretransformed->resize(cloud_pretransformed_xyz->size());
  for(int i = 0; i < cloud_pretransformed_xyz->size(); ++i) {
    (*cloud_pretransformed)[i].x = (*cloud_pretransformed_xyz)[i].x;
    (*cloud_pretransformed)[i].y = (*cloud_pretransformed_xyz)[i].y;
    (*cloud_pretransformed)[i].z = (*cloud_pretransformed_xyz)[i].z;
    (*cloud_pretransformed)[i].r = 'Î';
    (*cloud_pretransformed)[i].g = '!';
    (*cloud_pretransformed)[i].b = '!';
  }
  cloud_pretransformed_top->resize(cloud_pretransformed_xyz_t->size());
  for(int i = 0; i < cloud_pretransformed_xyz_t->size(); ++i) {
    (*cloud_pretransformed_top)[i].x = (*cloud_pretransformed_xyz_t)[i].x;
    (*cloud_pretransformed_top)[i].y = (*cloud_pretransformed_xyz_t)[i].y;
    (*cloud_pretransformed_top)[i].z = (*cloud_pretransformed_xyz_t)[i].z;
    (*cloud_pretransformed_top)[i].r = '0';
    (*cloud_pretransformed_top)[i].g = 'g';
    (*cloud_pretransformed_top)[i].b = '§';
  }

  //Filter the data first with Passthrough filter
  pass.setInputCloud (cloud_pretransformed);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.01, 4); //should be 4m 
  pass.filter (*cloud_filtered_pass);
  pass.setInputCloud (cloud_pretransformed_top);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.01, 4); //should be 4m 
  pass.filter (*cloud_filtered_pass_top);

  //Filter the data secondly with Voxelgrid filter to decrease size
  vgfilter.setInputCloud (cloud_filtered_pass);
  vgfilter.setLeafSize (0.04f, 0.04f, 0.04f);
  vgfilter.filter (*cloud_filtered_bot);
  vgfilter.setInputCloud (cloud_filtered_pass_top);
  vgfilter.setLeafSize (0.04f, 0.04f, 0.04f);
  vgfilter.filter (*cloud_filtered_top);

  //tf listener for coordinate transformation camera to drone
  tf_transformer_bottom();
  position_vector_bottom = T_B_world_b.translation();
  position_vector_bottom += T_B_color_bot.translation(); //+
  drone_height_bottom = position_vector_bottom[2];

  tf_transformer_top();
  position_vector_top = T_B_world_t.translation();
  position_vector_top += T_B_color_top.translation(); //+
  drone_height_top = position_vector_top[2];
 
  //Write point cloud from camera frame to world frame
  pcl::PointCloud<PointT>::Ptr cloud_transformed {new pcl::PointCloud<PointT>}; //wrt odometry
  pcl::PointCloud<PointT>::Ptr cloud_transformed_top {new pcl::PointCloud<PointT>}; //wrt odometry
  Eigen::Vector3d point_pre;
  Eigen::Vector3d point_after;

  cloud_transformed->resize(cloud_filtered_bot->size ());
  *cloud_transformed = *cloud_filtered_bot;
  cloud_transformed_top->resize(cloud_filtered_top->size ());
  *cloud_transformed_top = *cloud_filtered_top;

  cloud_bodyframe_b->resize(cloud_filtered_bot->size ());
  *cloud_bodyframe_b = *cloud_filtered_bot;
  cloud_bodyframe_t->resize(cloud_filtered_top->size ());
  *cloud_bodyframe_t = *cloud_filtered_top;

  Eigen::Vector3d trans_t = T_B_color_bot.translation();
  Eigen::Matrix3d rota_t = T_B_color_bot.rotation();;
  //std::cerr << trans_t;
  trans_t[0] += 0.015;
  trans_t[1] = -0.07;
  
  //Transform from camera to world frame
  //if (service_called == 1){
  for (int i = 0; i < cloud_filtered_bot->size(); ++i) { 
    point_pre << (*cloud_filtered_bot)[i].x,(*cloud_filtered_bot)[i].y,(*cloud_filtered_bot)[i].z;
    point_after = rota_t * point_pre + trans_t;
    (*cloud_bodyframe_b)[i].x = point_after[0];
    (*cloud_bodyframe_b)[i].y = point_after[1];
    (*cloud_bodyframe_b)[i].z = point_after[2];  
    point_after = T_B_world_b *point_after;
    (*cloud_transformed)[i].x = point_after[0];
    (*cloud_transformed)[i].y = point_after[1];
    (*cloud_transformed)[i].z = point_after[2];
  } 

  for (int i = 0; i < cloud_filtered_top->size(); ++i) { 
    point_pre << (*cloud_filtered_top)[i].x,(*cloud_filtered_top)[i].y,(*cloud_filtered_top)[i].z;
    point_after = T_B_color_top * point_pre;
    (*cloud_bodyframe_t)[i].x = point_after[0];
    (*cloud_bodyframe_t)[i].y = point_after[1];
    (*cloud_bodyframe_t)[i].z = point_after[2];  
    point_after = T_B_world_t *point_after;
    (*cloud_transformed_top)[i].x = point_after[0];
    (*cloud_transformed_top)[i].y = point_after[1];
    (*cloud_transformed_top)[i].z = point_after[2];
  }    

  cloud_->resize(cloud_transformed->size()+cloud_transformed_top->size());
  *cloud_ = *cloud_transformed_top+*cloud_transformed;
  writer_.write ("cloud_fused.pcd", *cloud_, false);
  std::cerr << "PointCloud after of fusing: " << cloud_->size () << " data points." << std::endl;
  
  //see if pole is in cloud
  cloud_input_func();

  auto stop = high_resolution_clock::now();
  auto duration = duration_cast<microseconds>(stop - start);
  double duration_now = duration.count()/1000;

  std::cerr << "Time Algorithm: " << duration_now << std::endl;

  return true;
}

void POLE_SEG::tf_transformer_bottom() {
  tf::StampedTransform transform;

  try {
    tf_listener_.waitForTransform("boreas/base_link", "bottom/color", ros::Time(0), ros::Duration(5.0));  
    tf_listener_.lookupTransform("boreas/base_link", "bottom/color", ros::Time(0), transform);      
    ROS_INFO_STREAM("[Cylinder segmentation] Found base_link to color transform!");  
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
  }
  tf::transformTFToEigen(transform, T_B_color_bot);
}

void POLE_SEG::tf_transformer_top() {
  tf::StampedTransform transform_top;

  try {
    tf_listener_.waitForTransform("boreas/base_link", "top/color", ros::Time(0), ros::Duration(5.0));  
    tf_listener_.lookupTransform("boreas/base_link", "top/color", ros::Time(0), transform_top);      
    ROS_INFO_STREAM("[Cylinder segmentation] Found base_link to color transform!");  
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
  }
  tf::transformTFToEigen(transform_top, T_B_color_top);
}

void POLE_SEG::tf_transformer_world_b() {
  tf::StampedTransform transform_2;

  try {
    tf_listener_.waitForTransform("vicon", "geranos_boreas", recorded_pc_->header.stamp, ros::Duration(5.0));  
    tf_listener_.lookupTransform("vicon", "geranos_boreas", recorded_pc_->header.stamp, transform_2);      
  }
  catch (tf::TransformException &ex) {
    std::cerr << "Error world b";
    ROS_ERROR("%s",ex.what());
  }

  tf::transformTFToEigen(transform_2, T_B_world_b);
}

void POLE_SEG::tf_transformer_world_t() {
  tf::StampedTransform transform_2;
  try {
    tf_listener_.waitForTransform("vicon", "geranos_boreas", recorded_pc_top_->header.stamp, ros::Duration(5.0));  
    tf_listener_.lookupTransform("vicon", "geranos_boreas", recorded_pc_top_->header.stamp, transform_2);      
  }
  catch (tf::TransformException &ex) {
    std::cerr << "Error world t";
    ROS_ERROR("%s",ex.what());
  }
  tf::transformTFToEigen(transform_2, T_B_world_t);
}

void POLE_SEG::tf_transformer_pole() { //geranos_pole_al,geranos_pole_wh or geranos_pole_pa
  tf::StampedTransform transform_2;
  try {
    tf_listener_.waitForTransform("vicon", "geranos_pole_al", recorded_pc_top_->header.stamp, ros::Duration(5.0));  
    tf_listener_.lookupTransform("vicon", "geranos_pole_al", recorded_pc_top_->header.stamp, transform_2);      
  }
  catch (tf::TransformException &ex) {
    std::cerr << "Error world t";
    ROS_ERROR("%s",ex.what());
  }
  tf::transformTFToEigen(transform_2, T_B_pole_);
}
///////////////////////////////////
///////////////////////////////////
///////////////////////////////////
///////////////////////////////////
///////////////////////////////////
///////////////////////////////////

void POLE_SEG::cloud_input_func(){

  pcl::PointCloud<PointT>::Ptr cloud_input {new pcl::PointCloud<PointT>};
  cloud_input->resize(cloud_->size ());
  *cloud_input = *cloud_;

  ROS_INFO_STREAM ("Cylinder is being segmented - start point");  
  /////////////////////////// Voxelgrid filter to reduce cloud size ///////////////////////////
  vgfilter.setInputCloud (cloud_input);
  vgfilter.setLeafSize (0.04f, 0.04f, 0.04f);
  vgfilter.filter (*cloud_filtered_voxel);
  std::cerr << "PointCloud after Voxelgrid filtering has: " << cloud_filtered_voxel->size () << " data points." << std::endl;

// Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered_voxel);
  ne.setKSearch (100); //Set the number of k nearest neighbors to use for the feature estimation (the higher, the less holes in plane visible)
  ne.compute (*cloud_normals);
  //writer_.write ("cloud_VGfiltered.pcd", *cloud_filtered_voxel, false);

//////////////////////////////////////////// Plane ///////////////////////////////////////////
  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PARALLEL_PLANE); //SACMODEL_NORMAL_PARALLEL_PLANE
  seg.setNormalDistanceWeight (0.05);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (10000);

  //Axis of normal plane vector (25 degrees looking down on Boreas)
  double angle_plane = 0.05; //~ 6 degree offset possible
  seg.setEpsAngle(angle_plane);
  Eigen::Vector3f axis_plane;
  axis_plane << 0.0, 0.0, 1.0;
  seg.setAxis (axis_plane);

  seg.setDistanceThreshold (0.1); //Distance to the model threshold (user given parameter).
  seg.setInputCloud (cloud_filtered_voxel);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane,*coefficients_plane);
  plane_coeff_ << coefficients_plane->values[0], coefficients_plane->values[1], coefficients_plane->values[2], coefficients_plane->values[3];

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered_voxel);
  extract.setIndices (inliers_plane);
  extract.setNegative (false); //false = normal filter behavior
  // Write the planar inliers to disk
  extract.filter (*cloud_plane);
  std::cerr << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << '\n' << std::endl;
  writer_.write ("cloud_plane.pcd", *cloud_plane, false);

  double mean = 0;
  for (auto& point: *cloud_plane) {
    mean += point.z/cloud_plane->size();
    if(point.z > plane_max_z_)
      plane_max_z_ = point.z;
  }

  mean_plane_z = mean;

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered2);
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);

  writer_.write ("cloud_no_plane.pcd", *cloud_filtered2, false);

////////////////////////////////////// Cylinder(s) ////////////////////////////////////////////
  anzahl_cylinder_ = 0; //number of possibly correct cylinders found

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC); 
  seg.setNormalDistanceWeight (0.15); //Set the relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between point normals and the plane normal.
  //maybe decrease a bit?? (setNormalDistanceWeight)
  seg.setMaxIterations (750000); //maybe even higher, however higher number -> more time...

  //Axis of cylinder parallel to normal plane vector
  double angle = 0.1; //~ 6 degree offset possible
  seg.setEpsAngle(angle);
  Eigen::Vector3f axis_cylinder;
  axis_cylinder << coefficients_plane->values[0], coefficients_plane->values[1], coefficients_plane->values[2];
  seg.setAxis (axis_cylinder); 
  seg.setDistanceThreshold (0.26); //inliner to model max 30cm away - double R_max (how close a point to model to be defined as inliner)
  seg.setRadiusLimits (0.03, 0.13); //D_hole is 25cm -> R_max = 12cm
  seg.setInputCloud (cloud_filtered2);
  seg.setInputNormals (cloud_normals2);

  //Coefficients and clouds needed for the cylinder segmentation
  pcl::ModelCoefficients::Ptr coefficients_cylinder_0 {new pcl::ModelCoefficients};
  pcl::ModelCoefficients::Ptr coefficients_cylinder_1 {new pcl::ModelCoefficients};
  pcl::ModelCoefficients::Ptr coefficients_cylinder_2 {new pcl::ModelCoefficients};
  pcl::ModelCoefficients::Ptr coefficients_cylinder_3 {new pcl::ModelCoefficients};
  pcl::ModelCoefficients::Ptr coefficients_cylinder_4 {new pcl::ModelCoefficients};
  pcl::ModelCoefficients::Ptr coefficients_cylinder_5 {new pcl::ModelCoefficients};
  pcl::ModelCoefficients::Ptr coefficients_cylinder_6 {new pcl::ModelCoefficients};
  pcl::PointCloud<PointT>::Ptr cloud_cylinder_0 {new pcl::PointCloud<PointT>};
  pcl::PointCloud<PointT>::Ptr cloud_cylinder_1 {new pcl::PointCloud<PointT>};
  pcl::PointCloud<PointT>::Ptr cloud_cylinder_2 {new pcl::PointCloud<PointT>};
  pcl::PointCloud<PointT>::Ptr cloud_cylinder_3 {new pcl::PointCloud<PointT>};
  pcl::PointCloud<PointT>::Ptr cloud_cylinder_4 {new pcl::PointCloud<PointT>};
  pcl::PointCloud<PointT>::Ptr cloud_cylinder_5 {new pcl::PointCloud<PointT>};
  pcl::PointCloud<PointT>::Ptr cloud_cylinder_6 {new pcl::PointCloud<PointT>};

  // Obtain the top 7 cylinder inliers and coefficients
  std::cerr << "Starting segmentation of Top 7 Cylinders!" << std::endl; 
  seg.segment_several (*inliers_cylinder_0,*inliers_cylinder_1,*inliers_cylinder_2,*inliers_cylinder_3,*inliers_cylinder_4,*inliers_cylinder_5,*inliers_cylinder_6,*coefficients_cylinder_0,*coefficients_cylinder_1,*coefficients_cylinder_2,*coefficients_cylinder_3,*coefficients_cylinder_4,*coefficients_cylinder_5,*coefficients_cylinder_6);

  //Write cylinder 0 inliers to disk
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder_0);
  extract.setNegative (false);
  extract.filter (*cloud_cylinder_0);

  //Write cylinder 1 inliers to disk
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder_1);
  extract.setNegative (false);
  extract.filter (*cloud_cylinder_1);

  //Write cylinder 2 inliers to disk
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder_2);
  extract.setNegative (false);
  extract.filter (*cloud_cylinder_2); 

  //Write cylinder 3 inliers to disk
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder_3);
  extract.setNegative (false);
  extract.filter (*cloud_cylinder_3);  

  //Write cylinder 4 inliers to disk
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder_4);
  extract.setNegative (false);
  extract.filter (*cloud_cylinder_4);  

  //Write cylinder 5 inliers to disk
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder_5);
  extract.setNegative (false);
  extract.filter (*cloud_cylinder_5);

  //Write cylinder 6 inliers to disk
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder_6);
  extract.setNegative (false);
  extract.filter (*cloud_cylinder_6);

  //this would be very very helpful to see if right cylinder
  //Pointcloud with all cylinders for testing
  pcl::PointCloud<PointT>::Ptr cylinders_before {new pcl::PointCloud<PointT>};

  all_cylinders_nothing_else_cloud->resize(cloud_cylinder_0->size() + cloud_cylinder_1->size() + cloud_cylinder_2->size() + cloud_cylinder_3->size() + cloud_cylinder_4->size() + cloud_cylinder_5->size() + cloud_cylinder_6->size());
  *all_cylinders_nothing_else_cloud = *cloud_cylinder_6 + *cloud_cylinder_5 + *cloud_cylinder_4 + *cloud_cylinder_3 + *cloud_cylinder_2 + *cloud_cylinder_1 + *cloud_cylinder_0;  
  
  writer_.write ("all_cylinders_nothing_else.pcd", *all_cylinders_nothing_else_cloud, false); 

  //check if each cloud is empty or not
  current_cylinder_ = 0;
  if (!cloud_cylinder_0->points.empty ()) {
    //writer_.write ("cloud_cylinder_0.pcd", *cloud_cylinder_0, false); 
    radius_[0] = coefficients_cylinder_0->values[6];
    cylinder_found_pole_ = cylinder_found_(cloud_cylinder_0,coefficients_cylinder_0);
    cylinder_can_be_pole_(cloud_cylinder_0);  
  }

  current_cylinder_ = 1;
  if (!cloud_cylinder_1->points.empty ()) {
    //writer_.write ("cloud_cylinder_1.pcd", *cloud_cylinder_1, false); 
    radius_[1] = coefficients_cylinder_1->values[6];
    cylinder_found_pole_ = cylinder_found_(cloud_cylinder_1,coefficients_cylinder_1);
    cylinder_can_be_pole_(cloud_cylinder_1);  
  }
  
  current_cylinder_ = 2;
  if (!cloud_cylinder_2->points.empty ()) {
    //writer_.write ("cloud_cylinder_2.pcd", *cloud_cylinder_2, false); 
    radius_[2] = coefficients_cylinder_2->values[6];
    cylinder_found_pole_ = cylinder_found_(cloud_cylinder_2,coefficients_cylinder_2);
    cylinder_can_be_pole_(cloud_cylinder_2);  
  }
  
  current_cylinder_ = 3;
  if (!cloud_cylinder_3->points.empty ()) {
    //writer_.write ("cloud_cylinder_3.pcd", *cloud_cylinder_3, false); 
    radius_[3] = coefficients_cylinder_3->values[6];
    cylinder_found_pole_ = cylinder_found_(cloud_cylinder_3,coefficients_cylinder_3);
    cylinder_can_be_pole_(cloud_cylinder_3);  
  }
  
  current_cylinder_ = 4;
  if (!cloud_cylinder_4->points.empty ()) {
    //writer_.write ("cloud_cylinder_4.pcd", *cloud_cylinder_4, false); 
    radius_[4] = coefficients_cylinder_4->values[6];
    cylinder_found_pole_ = cylinder_found_(cloud_cylinder_4,coefficients_cylinder_4);
    cylinder_can_be_pole_(cloud_cylinder_4);  
  }
  
  current_cylinder_ = 5;
  if (!cloud_cylinder_5->points.empty ()) {
    //writer_.write ("cloud_cylinder_5.pcd", *cloud_cylinder_5, false); 
    radius_[5] = coefficients_cylinder_5->values[6];
    cylinder_found_pole_ = cylinder_found_(cloud_cylinder_5,coefficients_cylinder_5);
    cylinder_can_be_pole_(cloud_cylinder_5);  
  }
  
  current_cylinder_ = 6;
  if (!cloud_cylinder_6->points.empty ()) {
    //writer_.write ("cloud_cylinder_6.pcd", *cloud_cylinder_6, false); 
    radius_[6] = coefficients_cylinder_6->values[6];
    cylinder_found_pole_ = cylinder_found_(cloud_cylinder_6,coefficients_cylinder_6);
    cylinder_can_be_pole_(cloud_cylinder_6);
  }

  std::cerr << "Total number of possibly correct cylinders: " << anzahl_cylinder_ << '\n' << std::endl;
} 

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////Cylinder condition functions////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

int POLE_SEG::cylinder_found_(const pcl::PointCloud<PointT>::Ptr &cloud_cylinder, pcl::ModelCoefficients::Ptr &coefficients_cylinder){  
  ///////////////////////// Bottom of pole in view //////////////////////////////////
  double tangens_alpha = 0.637; //32.5°
  double distance_to_pole = 4; //maximum distance from pole to camera due to passthrough filter
  Eigen::Vector3d top_cam_coord = position_vector_top; //coordinates from top camera


  //distance between pole and camera
  for (auto& point: *cloud_cylinder) {
    if ((sqrt((point.x-top_cam_coord[0])*(point.x-top_cam_coord[0]))+((point.y-top_cam_coord[1])*(point.y-top_cam_coord[1]))) < distance_to_pole) { 
      distance_to_pole = sqrt((point.x-top_cam_coord[0])*(point.x-top_cam_coord[0]))+((point.y-top_cam_coord[1])*(point.y-top_cam_coord[1]));
    }
  }

  std::cerr << "58°->46°: " << drone_height_top*tangens_alpha << std::endl;
  std::cerr << "distance to pole: " << distance_to_pole << std::endl;
  std::cerr << "32.5°: " << drone_height_top*0.637 << std::endl;

  if (distance_to_pole < drone_height_top*tangens_alpha) {
    std::cerr << "Bottom of pole not in camera view!" << std::endl;
    bottom_not_in_view = 1;
    min_height_ = distance_to_pole/tangens_alpha;
    return 1;
  }

  ///////////////////////// Pole stands on floor //////////////////////////////////
  //std::cerr << distance_to_pole << " " << drone_height*tangens_alpha << std::endl;
  for (auto& point: *cloud_cylinder) {
    if(point.z < plane_max_z_+0.1) { //less than 10cm away from plane
      ++pole_on_floor_;
      break;
    }
  }

  if (pole_on_floor_ == 0) {
    std::cerr << "Pole NOT on floor" << std::endl;
    return 2;
  }

  ///////////////////////// Length of Cylinder //////////////////////////////////
  double min_z_cy = 4;
  double max_z_cy = 0;
  for (auto& point: *cloud_cylinder) {
    if (point.z > max_z_cy)
      max_z_cy = point.z; 
    if (point.z < min_z_cy)
      min_z_cy = point.z;
  }
  length_cylinder_ = max_z_cy - mean_plane_z; //min_z_cy;
  //std::cerr << "Length of cylinder with mean: " << max_z_cy - mean_plane_z << std::endl; 

  ///////////////////////// Top of pole in view //////////////////////////////////
  double tangens_gamma = 1.5697; //57.5°
  if(length_cylinder_-0.03 > drone_height_bottom+distance_to_pole*tangens_gamma) {
    std::cerr << "Top of pole not in camera view!" << std::endl;
    top_not_in_view = 1;
    return 3;
  }

  ///////////////////////// Pole at edge of view //////////////////////////////////
  pcl::PointCloud<PointT>::Ptr cloud_bodyframe_after_t {new pcl::PointCloud<PointT>};
  pcl::PointCloud<PointT>::Ptr cloud_plane_bf {new pcl::PointCloud<PointT>};
  cloud_bodyframe_after_t->resize(cloud_cylinder->size());
  *cloud_bodyframe_after_t = *cloud_cylinder;
  cloud_plane_bf->resize(cloud_plane->size());
  *cloud_plane_bf = *cloud_plane;
  Eigen::Vector3d transf;
  Eigen::Vector3d transf_after;
  Eigen::Matrix3d rotation_t = T_B_world_t.rotation();
  Eigen::Matrix3d rotation_inverse_t = rotation_t.inverse();
  Eigen::Vector3d translation_t = T_B_world_t.translation();

  int i = 0;
  for (auto& point: *cloud_cylinder) { //only top camera shows ground
    transf << point.x, point.y, point.z;
    transf_after = rotation_inverse_t * (transf - translation_t);
    (*cloud_bodyframe_after_t)[i].x = transf_after[0];
    (*cloud_bodyframe_after_t)[i].y = transf_after[1];
    (*cloud_bodyframe_after_t)[i].z = transf_after[2];
    ++i;
  }

  i = 0;
  for (auto& point: *cloud_plane) { //only top camera shows ground
    transf << point.x, point.y, point.z;
    transf_after = rotation_inverse_t * (transf - translation_t);
    (*cloud_plane_bf)[i].x = transf_after[0];
    (*cloud_plane_bf)[i].y = transf_after[1];
    (*cloud_plane_bf)[i].z = transf_after[2];
    ++i;
  }
  writer_.write ("cloud_bodyframe_after_t.pcd", *cloud_plane_bf, false);

  double tanges_yaw_max_cy = atan2((*cloud_bodyframe_after_t)[0].y,(*cloud_bodyframe_after_t)[0].x);
  double tanges_yaw_min_cy = atan2((*cloud_bodyframe_after_t)[0].y,(*cloud_bodyframe_after_t)[0].x);

  for(int i = 0; i < cloud_cylinder->size(); ++i) {
    if(atan2((*cloud_bodyframe_after_t)[i].y,(*cloud_bodyframe_after_t)[i].x) < tanges_yaw_min_cy){
      tanges_yaw_min_cy = atan2((*cloud_bodyframe_after_t)[i].y,(*cloud_bodyframe_after_t)[i].x);
    } 
    if(atan2((*cloud_bodyframe_after_t)[i].y,(*cloud_bodyframe_after_t)[i].x) > tanges_yaw_max_cy){
      tanges_yaw_max_cy = atan2((*cloud_bodyframe_after_t)[i].y,(*cloud_bodyframe_after_t)[i].x);
    } 
  }
  //std::cerr << tanges_yaw_max_cy << " " << tanges_yaw_min_cy << std::endl;

  double bigger = 0;
  double smaller = 0;
  double not_freestanding = 0;
  for(auto& point: *cloud_plane_bf) {
    if(atan2(point.y,point.x) > tanges_yaw_max_cy) {
      //point.r = 'ü';
      ++bigger;
    }
    if(atan2(point.y,point.x) < tanges_yaw_min_cy) {
      //point.b = 'ü';
      ++smaller;
    }
    if(bigger/cloud_plane_bf->size() > 0.075 && smaller/cloud_plane_bf->size() > 0.075)
      break;
    //if distance between 25 and 75.5cm -> No pole 
  /*  if(sqrt(((*cloud_bodyframe_after_t)[0].x-point.x)*((*cloud_bodyframe_after_t)[0].x-point.x)+((*cloud_bodyframe_after_t)[0].y-point.y)*((*cloud_bodyframe_after_t)[0].y-point.y)) > 0.25 && sqrt(((*cloud_bodyframe_after_t)[0].x-point.x)*((*cloud_bodyframe_after_t)[0].x-point.x)+((*cloud_bodyframe_after_t)[0].y-point.y)*((*cloud_bodyframe_after_t)[0].y-point.y)) < 0.755)
      not_freestanding = 1;
    if(not_freestanding == 1)
      break;*/
  }

  std::cerr << bigger << " bigger/smaller " << smaller << std::endl;

  if(not_freestanding == 1) { 
    std::cerr << "Pole not freestanding!" << std::endl;
    return 300;
  }
  if(bigger/cloud_plane_bf->size() < 0.075) { //outer 7.5%
    std::cerr << "Pole at the left edge of camera view!" << std::endl;
    pole_in_edge_up = 1;
    return 5;
  }
  if(smaller/cloud_plane_bf->size() < 0.075) { //outer 7.5%
    std::cerr << "Pole at the right edge of camera view!" << std::endl;
    pole_in_edge_low = 1;
    return 4;
  }

  ///////////////////////// Datapoints - pole //////////////////////////////////
  if(cloud_cylinder->size()/length_cylinder_ <  3.14*radius_[current_cylinder_]*100*100/16*0.75) { //pi*r*l/16 [cm^2] 0.75 is safety factor..
    //std::cerr << "cloud_cylinder->size()/length: " << cloud_cylinder->size()/length_cylinder_ << std::endl;
    //std::cerr << "3.14*radius_[current_cylinder_]*100*100/16: " << 3.14*radius_[current_cylinder_]*100*100/16*0.75 << std::endl;
    return 6;
  }

  ///////////////////////// CoM of pole //////////////////////////////////
  double lambda = (max_z_cy-length_cylinder_/2-coefficients_cylinder->values[2])/coefficients_cylinder->values[5];
  double xx = coefficients_cylinder->values[0] + lambda*coefficients_cylinder->values[3];
  double yy = coefficients_cylinder->values[1] + lambda*coefficients_cylinder->values[4];
  double zz = coefficients_cylinder->values[2] + lambda*coefficients_cylinder->values[5];
  CoM_pole_.x = xx; //basically the same as the coefficients (+-2mm)
  CoM_pole_.y = yy;
  CoM_pole_.z = zz;

  ++anzahl_cylinder_; //current number of cylinder that could be the pole
  return 10;
}


void POLE_SEG::cylinder_can_be_pole_(const pcl::PointCloud<PointT>::Ptr &cloud_cylinder){ 
  if (cylinder_found_pole_ == 1) {
    std::cerr << "Cylinder number " << current_cylinder_ << ": Bottom of pole not in view!"  << '\n' <<  std::endl;
  }
  else if (cylinder_found_pole_ == 2) {
    std::cerr << "Cylinder Number " << current_cylinder_ <<" is not on the floor." << '\n' << std::endl;
  }
  else if (cylinder_found_pole_ == 3) {
    std::cerr << "Cylinder number " << current_cylinder_ << ": Top of pole not in view!"  << '\n' <<  std::endl;
  }
  else if (cylinder_found_pole_ == 4) {
    std::cerr << "Cylinder number " << current_cylinder_ << ": Pole on left edge of view!"  << '\n' <<  std::endl;
  }
  else if (cylinder_found_pole_ == 5) {
    std::cerr << "Cylinder number " << current_cylinder_ << ": Pole on right edge of view!"  << '\n' <<  std::endl;
  }
  else if (cylinder_found_pole_ == 6) {
    std::cerr << "Cylinder number " << current_cylinder_ << " does not have enough datapoints for the found model!"  << '\n' <<  std::endl;
  }
  else if (cylinder_found_pole_ == 10) {
    std::cerr << "Cylinder Length: " << length_cylinder_ << " m" <<  std::endl;
    //std::cerr << "Cylinder Number: " << current_cylinder_ << std::endl;
    std::cerr << "Cylinder Radius: " << radius_[current_cylinder_]*100 << " cm" << std::endl;
    std::cerr << "CoM: " << CoM_pole_.x << " " << CoM_pole_.y << " " << CoM_pole_.z << std::endl;
    //std::cerr << "#Inlier: " <<  cloud_cylinder->size() << std::endl;
    //std::cerr << "#Outlier: " << cloud_filtered2->size()-cloud_cylinder->size() <<  '\n' << std::endl;
    tf_transformer_pole();
    Eigen::Vector3d pole_pos_CoM = T_B_pole_.translation();
    std::cerr << "Vicon Pole: " << pole_pos_CoM <<  '\n' << std::endl;
    pole_found_[current_cylinder_] = true;
  }

  //new_view_point_trajectory();

  //do it again 
  //Reset to 0 for next cylinder
  cylinder_found_pole_ = 0;
} 


////////////Send coordinates of pole to trajectory ///////////////////
void POLE_SEG::new_view_point_trajectory() {
  Eigen::Matrix3d rotational_matrix = T_B_world_b.rotation();
  Eigen::Matrix3d rotational_matrix_t = T_B_world_t.rotation();
  Eigen::Vector3d current_location = T_B_world_b.translation();
  double current_yaw = atan2(rotational_matrix(1,0),rotational_matrix(0,0));
  double current_yaw_t = atan2(rotational_matrix_t(1,0),rotational_matrix_t(0,0));

  if(bottom_not_in_view == 1) {
    //trajectory lower so bottom is in view
    //current position but z = min_height_;
    service_response[0] = current_location[0];
    service_response[1] = current_location[1]; 
    service_response[2] = min_height_;
    service_response[3] = current_yaw; 
    std::cerr << "new trajectory point needed" << std::endl;
  }
  else if(top_not_in_view == 1) {
    //trajectory +0.5m
    //current position but z += 0.5m;
    service_response[0] = current_location[0]; 
    service_response[1] = current_location[1]; 
    service_response[2] = current_location[2]+0.5; 
    service_response[3] = current_yaw; 
    std::cerr << "new trajectory point needed" << std::endl;
  }
  else if(pole_in_edge_up == 1) {
    //trajectory +45°
    //current position but yaw += 45°;
    service_response[0] = current_location[0]; 
    service_response[1] = current_location[1]; 
    service_response[2] = 1; 
    service_response[3] = current_yaw+0.785398;
    std::cerr << "new trajectory point needed" << std::endl;
  }
  else if(pole_in_edge_low == 1) {
    //trajectory -45°
    //current position but yaw -= 45°;
    service_response[0] = current_location[0];
    service_response[1] = current_location[1];
    service_response[2] = 1; 
    service_response[3] = current_yaw-0.785398;
    std::cerr << "new trajectory point needed" << std::endl;
  }
  else if(pole_found_[current_cylinder_] == 1) { //Pole found
    std::cerr << "Pole found :)" << std::endl;
    service_response[0] = CoM_pole_.x;
    service_response[1] = CoM_pole_.y;
    service_response[2] = CoM_pole_.z; 
    service_response[3] = length_cylinder_;
  }
  else { //No pole found
    std::cerr << "new trajectory point needed" << std::endl;
    service_response[0] = current_location[0]; 
    service_response[1] = current_location[1]; 
    service_response[2] = 1;
    service_response[3] = current_yaw+0.785398;
  }
}


// Main ROS method
int main(int argc, char **argv) {
    
  // Initialize the node and set the name
  ros::init(argc, argv, "adder_server");
   
  // Create the main access point for the node
  // This piece of code enables the node to communicate with the ROS system.
  ros::NodeHandle nh;
  POLE_SEG pole_seg;
  ros::ServiceServer service_;

  // Create the service and advertise it to the ROS computational network
  ros::Subscriber sub2 = nh.subscribe("/boreas/camera/bottom/depth/color/points", 1, &POLE_SEG::Callback, &pole_seg);
  ros::Subscriber sub3 = nh.subscribe("/boreas/camera/top/depth/color/points", 1, &POLE_SEG::Callback_top, &pole_seg);

  pole_seg.client_ = nh.serviceClient<std_srvs::Empty>("record_and_fusing_pointcloud");
  ros::ServiceServer service = nh.advertiseService("/bachelor_thesis/pole_detection_only", &POLE_SEG::Service_real, &pole_seg);
  pole_seg.client2_ = nh.serviceClient<std_srvs::Empty>("/bachelor_thesis/segmentation_fused_cloud");



  // Keep processing information over and over again
  ros::spin();
 
  // Program completed successfully
  return 0;
}