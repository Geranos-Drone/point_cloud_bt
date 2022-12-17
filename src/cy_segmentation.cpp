#include <bachelor_thesis/cy_segmentation.h>

using namespace std::chrono;

//rosrun pcl_ros pcd_to_pointcloud <file_name>.pcd 0.1 _frame_id:=/map -> pcd file can be viewed in rviz as pc

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointXYZ;


//switch_param_client_ = nh_.serviceClient<std_srvs::Empty>("impedance_module/switch_control_params");
//        PoleDet_service = rospy.ServiceProxy('/bachelor_thesis/detection_node', PoleDetetion)
//pole_det = nh_.serviceClient<std_srvs::Empty>("bachelor_thesis/detection_node")

void POLE_SEG::callback_(const sensor_msgs::PointCloud2::ConstPtr& cloud_pcd){                                
  std_srvs::Empty pole_det;
  ROS_WARN_STREAM("In callback");
  
  pole_detection_ = nh_.serviceClient<std_srvs::Empty>("bachelor_thesis/detection_node");
  
  if (pole_detection_.call(pole_det)) 
  {
    std::cerr << "yes";
    ROS_WARN_STREAM("Service received!");
  }

  if(!callback_called_once_) {
    segmentation_(cloud_pcd);
  }
  else {
    return;
  }

}
//function called from subscriber
//first converts sensor_message to point cloud
//Afterwards pland and cylinder segmentation
void POLE_SEG::segmentation_(const sensor_msgs::PointCloud2::ConstPtr& cloud_pcd){                                

  ROS_ERROR_STREAM ("Cylinder is being segmented - start point");

  //converts sensor_msgs to pointcloud
  pcl_conversions::toPCL(*cloud_pcd,pcl_pc2); //Googlen
  pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
  std::cerr << "PointCloud has: " << cloud->size () << " data points." << std::endl;

  if(cloud->points.empty ()) { //if received pcd_cloud empty end callback function and wait until cloud is not empty anymore
    ROS_ERROR("Empty cloud");
    //OUTPUT SOMETHING
    return;
  }

  /////////////////////////// Passthrough Filter ///////////////////////////
  //Filtering data -> deleting data with more than 4m distance due to depth error
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.01, 4); 
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->size () << " data points." << std::endl;

  if(cloud_filtered->points.empty ()) { //if received pcd_cloud empty end callback function and wait until cloud is not empty anymore
    ROS_ERROR("Empty cloud");
    //OUTPUT SOMETHING
    return; // (point_xyz);
  }

/////////////////////////// Voxelgrid filter to reduce cloud size (~number is cut in half) ///////////////////////////
  vgfilter.setInputCloud (cloud_filtered);
  vgfilter.setLeafSize (0.025f, 0.025f, 0.025f);
  vgfilter.filter (*cloud_filtered_voxel);
  std::cerr << "PointCloud after Voxelgrid filtering has: " << cloud_filtered_voxel->size () << " data points." << std::endl;

// Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered_voxel);
  ne.setKSearch (100); //Set the number of k nearest neighbors to use for the feature estimation (the higher, the less holes in plane visible)
  ne.compute (*cloud_normals);
  writer_.write ("VG_filter_test.pcd", *cloud_filtered_voxel, false);

//////////////////////////////////////////// Plane ///////////////////////////////////////////
  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PARALLEL_PLANE); //SACMODEL_NORMAL_PARALLEL_PLANE
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (10000);

  //Axis of normal plane vector (25 degrees looking down on Boreas)
  double angle_plane = 0.05;
  seg.setEpsAngle(angle_plane);
  Eigen::Vector3f axis_plane;
  axis_plane << 0.0, 1, 0.2; //Simulation axis
  seg.setAxis (axis_plane);

  seg.setDistanceThreshold (0.1); //Distance to the model threshold (user given parameter).
  seg.setInputCloud (cloud_filtered_voxel);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane,*coefficients_plane);
  std::cerr << "okay 4";

  plane_coeff_ << coefficients_plane->values[0], coefficients_plane->values[1], coefficients_plane->values[2], coefficients_plane->values[3];
  std::cerr << "plane_coeff_" << plane_coeff_;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered_voxel);
  extract.setIndices (inliers_plane);
  extract.setNegative (false); //false = normal filter behavior
  // Write the planar inliers to disk
  extract.filter (*cloud_plane);
  std::cerr << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << '\n' << std::endl;
  writer_.write ("plane_test.pcd", *cloud_plane, false);
  
  if(cloud_plane->points.empty ()) { //if received pcd_cloud empty end callback function and wait until cloud is not empty anymore
    ROS_ERROR("Empty plane-cloud");
    //OUTPUT SOMETHING
    return; //(point_xyz);
  }
  
  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered2);
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);

////////////////////////////////////// Cylinder(s) ////////////////////////////////////////////
  anzahl_cylinder_ = 0; //number of possibly correct cylinders found

  if(cloud_filtered2->points.empty ()) { //if received pcd_cloud empty end callback function and wait until cloud is not empty anymore
    ROS_ERROR("Empty cloud_filtered2-cloud");
    //OUTPUT SOMETHING
    return; // (point_xyz);
  }

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC); 
  seg.setNormalDistanceWeight (0.2); //Set the relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between point normals and the plane normal.
  //maybe decrease a bit?? (setNormalDistanceWeight)
  seg.setMaxIterations (500000); //maybe even higher, however higher number -> more time...

  //Axis of cylinder parallel to normal plane vector
  double angle = 0.1; //~ 2degree offset possible
  seg.setEpsAngle(angle);
  Eigen::Vector3f axis_cylinder;
  axis_cylinder << coefficients_plane->values[0], coefficients_plane->values[1], coefficients_plane->values[2];
  seg.setAxis (axis_cylinder); 
  seg.setDistanceThreshold (0.26); //inliner to model max 26cm away - double R_max (how close a point to model to be defined as inliner)
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
  //Used for original pcl, DELETE afterwards again please
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

  std::cerr << cloud_cylinder_0->size() << " "<< cloud_cylinder_1->size() << " "<< cloud_cylinder_2->size() << " "<< cloud_cylinder_3->size() << " "<< cloud_cylinder_4->size() << " "<< cloud_cylinder_5->size() << " "<< cloud_cylinder_6->size();

  //this would be very very helpful to see if right cylinder
  //Pointcloud with all cylinders for testing
  pcl::PointCloud<PointT>::Ptr all_cylinders_nothing_else_cloud {new pcl::PointCloud<PointT>};
  all_cylinders_nothing_else_cloud->resize(cloud_cylinder_0->size() + cloud_cylinder_1->size() + cloud_cylinder_2->size() + cloud_cylinder_3->size() + cloud_cylinder_4->size() + cloud_cylinder_5->size() + cloud_cylinder_6->size());
  *all_cylinders_nothing_else_cloud = *cloud_cylinder_6 + *cloud_cylinder_5 + *cloud_cylinder_4 + *cloud_cylinder_3 + *cloud_cylinder_2 + *cloud_cylinder_1 + *cloud_cylinder_0;
  writer_.write ("all_cylinders_nothing_else.pcd", *all_cylinders_nothing_else_cloud, false); 
  

  //check if each cloud is empty or not
  current_cylinder_ = 0;
  if (!cloud_cylinder_0->points.empty ()) {
    writer_.write ("cloud_cylinder_0.pcd", *cloud_cylinder_0, false); 
    radius_[0] = coefficients_cylinder_0->values[6];
    cylinder_found_pole_ = cylinder_found_(cloud_cylinder_0,coefficients_cylinder_0);
    point_minmax_0_ = point_minmax_;
  }
  cylinder_can_be_pole_(cloud_cylinder_0);  

  current_cylinder_ = 1;
  if (!cloud_cylinder_1->points.empty ()) {
    writer_.write ("cloud_cylinder_1.pcd", *cloud_cylinder_1, false); 
    radius_[1] = coefficients_cylinder_1->values[6];
    cylinder_found_pole_ = cylinder_found_(cloud_cylinder_1,coefficients_cylinder_1);
    point_minmax_1_ = point_minmax_;
  }
  cylinder_can_be_pole_(cloud_cylinder_1);  
  
  current_cylinder_ = 2;
  if (!cloud_cylinder_2->points.empty ()) {
    writer_.write ("cloud_cylinder_2.pcd", *cloud_cylinder_2, false); 
    radius_[2] = coefficients_cylinder_2->values[6];
    cylinder_found_pole_ = cylinder_found_(cloud_cylinder_2,coefficients_cylinder_2);
    point_minmax_2_ = point_minmax_;
  }
  cylinder_can_be_pole_(cloud_cylinder_2);  
  
  current_cylinder_ = 3;
  if (!cloud_cylinder_3->points.empty ()) {
    writer_.write ("cloud_cylinder_3.pcd", *cloud_cylinder_3, false); 
    radius_[3] = coefficients_cylinder_3->values[6];
    cylinder_found_pole_ = cylinder_found_(cloud_cylinder_3,coefficients_cylinder_3);
    point_minmax_3_ = point_minmax_;
  }
  cylinder_can_be_pole_(cloud_cylinder_3);  
  
  current_cylinder_ = 4;
  if (!cloud_cylinder_4->points.empty ()) {
    writer_.write ("cloud_cylinder_4.pcd", *cloud_cylinder_4, false); 
    radius_[4] = coefficients_cylinder_4->values[6];
    cylinder_found_pole_ = cylinder_found_(cloud_cylinder_4,coefficients_cylinder_4);
    point_minmax_4_ = point_minmax_;
  }
  cylinder_can_be_pole_(cloud_cylinder_4);  
  
  current_cylinder_ = 5;
  if (!cloud_cylinder_5->points.empty ()) {
    writer_.write ("cloud_cylinder_5.pcd", *cloud_cylinder_5, false); 
    radius_[5] = coefficients_cylinder_5->values[6];
    cylinder_found_pole_ = cylinder_found_(cloud_cylinder_5,coefficients_cylinder_5);
    point_minmax_5_ = point_minmax_;
  }
  cylinder_can_be_pole_(cloud_cylinder_5);  
  
  current_cylinder_ = 6;
  if (!cloud_cylinder_6->points.empty ()) {
    writer_.write ("cloud_cylinder_6.pcd", *cloud_cylinder_6, false); 
    radius_[6] = coefficients_cylinder_6->values[6];
    cylinder_found_pole_ = cylinder_found_(cloud_cylinder_6,coefficients_cylinder_6);
  }
  cylinder_can_be_pole_(cloud_cylinder_6);  


  //tf
  //color_only_cylinders_(cloud_cylinder_0,cloud_cylinder_1,cloud_cylinder_2,cloud_cylinder_3,cloud_cylinder_4,cloud_cylinder_5,cloud_cylinder_6);


  std::cerr << "Total number of cylinders found: " << cylinder_number_ << std::endl;
  std::cerr << "Total number of possibly correct cylinders: " << anzahl_cylinder_ << '\n' << std::endl;
  callback_called_once_ = 1;
} 



//all other function

int POLE_SEG::cylinder_found_(const pcl::PointCloud<PointT>::Ptr &cloud_cylinder, pcl::ModelCoefficients::Ptr &coefficients_cylinder){  
  ///////////////////////// Length of Pole //////////////////////////////////
  Eigen::Vector3f real_axis_cylinder; //axis of cylinder
  real_axis_cylinder << coefficients_cylinder->values[3], coefficients_cylinder->values[4], coefficients_cylinder->values[5];
  float length_absolute = 0; //absolute length of cylinder
  Eigen::Vector3f vector_points_distance; //distance vector of the two most extreme points

  if (real_axis_cylinder[2] > 0) { //z coordinate must be negative due to coordinate system
      real_axis_cylinder *= -1;
  }
  
  const auto& point = cloud_cylinder->points; //all points on cylinder
  Eigen::Vector3f distance_vec; //distance between 2 points

  float ebene;
  float ebene_closer = 1; //1m

  //distance between all points -> maximum projected is absolute length of cylinder
  for (int i = 0; i < cloud_cylinder-> size (); ++i){
    ebene = point[i].x*coefficients_cylinder->values[3] + point[i].y*coefficients_cylinder->values[4] + point[i].z*coefficients_cylinder->values[5];
    if (abs(abs(ebene) - abs(plane_coeff_[3])) < ebene_closer) {
      ebene_closer = abs(ebene - plane_coeff_[3]);
      point_minmax_= point[i];
      pointminmax_ = i;
    }
    for (int j = i+1; j < cloud_cylinder-> size (); ++j){
      distance_vec << point[i].x - point[j].x, point[i].y - point[j].y, point[i].z - point[j].z;
      float distance = sqrt(distance_vec.dot(distance_vec)); //absolute value of distance vector between the two points
      if (distance > length_absolute) {
        length_absolute = distance;
        vector_points_distance = distance_vec;
      }
    }
  } 

  float cos_real = vector_points_distance.dot(real_axis_cylinder)/length_absolute;
  length_axis_real_ = abs(length_absolute * cos_real);

  ///////////////////////// Found cylinder too similar to previous found cylinders? //////////////////////////////////

  if(abs(abs(point_minmax_0_.x - point_minmax_.x) + abs(point_minmax_0_.y - point_minmax_.y) + abs(point_minmax_0_.z - point_minmax_.z)) < 0.5) {
    ++cylinder_number_;
    return 1;
  }
  if(abs(abs(point_minmax_1_.x - point_minmax_.x) + abs(point_minmax_1_.y - point_minmax_.y) + abs(point_minmax_1_.z - point_minmax_.z)) < 0.5) {
    ++cylinder_number_;
    return 1;
  }
  if(abs(abs(point_minmax_2_.x - point_minmax_.x) + abs(point_minmax_2_.y - point_minmax_.y) + abs(point_minmax_2_.z - point_minmax_.z)) < 0.5) {
    ++cylinder_number_;
    return 1;
  }
  if(abs(abs(point_minmax_3_.x - point_minmax_.x) + abs(point_minmax_3_.y - point_minmax_.y) + abs(point_minmax_3_.z - point_minmax_.z)) < 0.5) {
    ++cylinder_number_;
    return 1;
  }
  if(abs(abs(point_minmax_4_.x - point_minmax_.x) + abs(point_minmax_4_.y - point_minmax_.y) + abs(point_minmax_4_.z - point_minmax_.z)) < 0.5) {
    ++cylinder_number_;
    return 1;
  }
  if(abs(abs(point_minmax_5_.x - point_minmax_.x) + abs(point_minmax_5_.y - point_minmax_.y) + abs(point_minmax_5_.z - point_minmax_.z)) < 0.5) {
    ++cylinder_number_;
    return 1;
  }

    ///////////////////////// Pole must be freestanding //////////////////////////////////
    //Amount of points around pole in radius of 1m must be less than 1.5x the amount of data poins (-> excludes outliners)
    pcl::PointCloud<PointXYZ>::Ptr cloud_freestanding (new pcl::PointCloud<PointXYZ>);
    cloud_freestanding->resize(cloud_filtered2->size());

    for (int i = 0; i < cloud_filtered2->size (); ++i){
      (*cloud_freestanding)[i].x = (*cloud_filtered2)[i].x;
      (*cloud_freestanding)[i].y = (*cloud_filtered2)[i].y;
      (*cloud_freestanding)[i].z = (*cloud_filtered2)[i].z;
    }
    
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud_freestanding);
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    radius_kdtree_ = 0.5;
    pcl::PointXYZ searchPoint; //point in middle of pole
    searchPoint.x = point_minmax_.x + 0.5*length_axis_real_*real_axis_cylinder[0];
    searchPoint.y = point_minmax_.y + 0.5*length_axis_real_*real_axis_cylinder[1];
    searchPoint.z = point_minmax_.z + 0.5*length_axis_real_*real_axis_cylinder[2];

    //std::cerr << searchPoint;
    //std::cerr << kdtree.radiusSearch (searchPoint, radius_kdtree_, pointIdxRadiusSearch, pointRadiusSquaredDistance) << " " << 1.25*cloud_cylinder-> size ();

    //writer_.write ("cloud_freestanding_kdtree.pcd", *cloud_freestanding, false);

    if (kdtree.radiusSearch (searchPoint, radius_kdtree_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 1.25*cloud_cylinder-> size ())
    {
      ++cylinder_number_;
      return 4;
    } 

  
  ///////////////////////// Pole on the Plane/Floor? //////////////////////////////////
  float abs_distance; //distance between cylinderpoint and plane
  Eigen::Vector3f point_on_cylinder;

  point_on_cylinder << point_minmax_.x, point_minmax_.y, point_minmax_.z;
  Eigen::Vector3f axis_plane_3;
  axis_plane_3 << plane_coeff_[0], plane_coeff_[1], plane_coeff_[2];
  abs_distance = (point_on_cylinder.dot(axis_plane_3) + plane_coeff_[3])/sqrt(axis_plane_3.dot(axis_plane_3));

/*  if (abs(abs_distance) > 0.06) { //if plane more than 6cm away
    std::cerr << abs(abs_distance);
    ++cylinder_number_;
    return 2;
  } */

  ///////////////////////// Do the found coefficients add up to Cylinder Model? //////////////////////////////////
  //Datapoints must be at least 1/6 of a cylinder (according to found radius and length)
  if((cloud_cylinder->size ()/length_axis_real_) < 0.45*(3.14*100*coefficients_cylinder->values[6]*100/4)) {
    ++cylinder_number_;
    std::cerr << cloud_cylinder->size ()/length_axis_real_ << " " << 0.45*(3.14*100*coefficients_cylinder->values[6]*100/4);
    return 3;
  }

//////////////////////////////// All criteria are fulfilled //////////////////////////////////////

  //point on top of pole
  point_ontop_.x = point_minmax_.x + length_axis_real_*real_axis_cylinder[0];
  point_ontop_.y = point_minmax_.y + length_axis_real_*real_axis_cylinder[1];
  point_ontop_.z = point_minmax_.z + length_axis_real_*real_axis_cylinder[2];

  ++cylinder_number_;
  ++anzahl_cylinder_; //current number of cylinder that could be the pole
  return 10;
}


void POLE_SEG::cylinder_can_be_pole_(const pcl::PointCloud<PointT>::Ptr &cloud_cylinder){ 
  if (cylinder_found_pole_ == 0) {
    std::cerr << "Cylinder Number " << current_cylinder_ <<" was not found." << '\n' << std::endl;
  }
  else if (cylinder_found_pole_ == 1) {
    std::cerr << "Cylinder Number " << current_cylinder_ <<" too similar to previous found cylinder." << '\n' << std::endl;
    /*RED
    cylinder_red_or_green_(cylinder_number) = 0;
    for (auto& point: *cloud_cylinder) { 
      point.r = 'ü';
      point.g = '!';
      point.b = '!';
    } */
  }
  else if (cylinder_found_pole_ == 2) {
    std::cerr << "Cylinder Number " << current_cylinder_ <<" is not on the floor." << '\n' << std::endl;
    /*RED
    cylinder_red_or_green_(cylinder_number) = 0;
    for (auto& point: *cloud_cylinder) { 
      point.r = 'ü';
      point.g = '!';
      point.b = '!';
    }  */
  }
  else if (cylinder_found_pole_ == 3) {
    std::cerr << "Cylinder number " << current_cylinder_ << " does not have enough datapoints for the found model!"  << '\n' <<  std::endl;
    /*RED
    cylinder_red_or_green_(cylinder_number) = 0;
    for (auto& point: *cloud_cylinder) { 
      point.r = 'ü';
      point.g = '!';
      point.b = '!';
    } */
  }
  else if (cylinder_found_pole_ == 4) {
    std::cerr << "Cylinder Number " << current_cylinder_ <<" is not freestanding! (Radius used: " << radius_kdtree_ << "m)" << '\n' << std::endl;
    /*RED
    cylinder_red_or_green_(cylinder_number) = 0;
    for (auto& point: *cloud_cylinder) { 
      point.r = 'ü';
      point.g = '!';
      point.b = '!';
    } */
  }
  else if (cylinder_found_pole_ == 10) { //Pole can be the cylinder
    std::cerr << "Cylinder Number: " << current_cylinder_ << std::endl;
    std::cerr << "Cylinder Radius: " << radius_[current_cylinder_]*100 << " cm" << std::endl;
    std::cerr << "Cylinder Length: " << length_axis_real_ << " m" << std::endl;
    std::cerr << "Cylinder Point (Camera Frame): " << point_minmax_.x << ", " << point_minmax_.y << ", " << point_minmax_.z << std::endl;
    tf_transformer(point_ontop_); //

    //GREEN    
    cylinder_red_or_green_[current_cylinder_] = 1;
    for (auto& point: *cloud_cylinder) { 
      point.r = '!';
      point.g = 'ü';
      point.b = '!';
    }
    auto& point = cloud_cylinder->points;

    point[pointminmax_].r = 'ü';
    point[pointminmax_].g = '!';
    point[pointminmax_].b = '!';

    std::string cn_string = std::to_string(cylinder_number_);//current number of cylinder as string
    writer_.write ("cylinder_colored_" + cn_string + ".pcd", *cloud_cylinder, false);

  }

  //writer_.write ("cylinder_colored_" + cn_string + ".pcd", *cloud_cylinder, false);
  
  //Reset to 0 for next cylinder
  cylinder_found_pole_ = 0;
} 

void POLE_SEG::tf_transformer(const pcl::PointXYZRGB& point_on_pole){
  //tf listener for coordinate transformation
    ros::NodeHandle node;
    ros::Rate rate(10.0);
    Eigen::Vector3d point;
    Eigen::Vector3d point_final;
    point << point_on_pole.x, point_on_pole.y, point_on_pole.z;
    int ii = 0;

    //while (node.ok()){
      tf::StampedTransform transform;

    try{
      tf_listener_.waitForTransform("boreas/base_link", "color", ros::Time(0), ros::Duration(5.0));  
      tf_listener_.lookupTransform("boreas/base_link", "color", ros::Time(0), transform);
      
      ROS_INFO_STREAM("[Cylinder segmentation] Found base_link to color transform!");  

    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      //ros::Duration(1.0).sleep();
      //continue;
    }
    tf::transformTFToEigen(transform, T_B_color_);
    //std::cerr << T_B_color_.matrix();
    Eigen::Matrix3d transform_matrix;

    transform_matrix(0,0) = T_B_color_(0,0);
    point_final = T_B_color_ * point;

    std::cerr << T_B_color_.matrix();

    std::cerr << "Cylinder Point (Body Frame): " << point_final[0] << ", " << point_final[1] << ", " << point_final[2] << '\n' << std::endl;

  //}
} 

int main(int argc, char** argv) {
  ros::init(argc, argv,"test_node");
  ros::NodeHandle nh;
  
  POLE_SEG pole_seg;
  ros::Rate r(10); // 10 hz

  //ros::Subscriber sub = nh.subscribe("input", 10, &POLE_SEG::callback_, &pole_seg);

  // Create a ROS publisher for the output point cloud (eg z Koordinate)
  //pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin, don't exit program
  ros::spin();

  return 0;

}