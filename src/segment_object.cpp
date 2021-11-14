#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/pcl_config.h>

#include <math.h>
#include <robotics_labs/BoxTarget.h>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "visualization_msgs/Marker.h"
#include "simple_grasping/shape_extraction.h"
#include "shape_msgs/SolidPrimitive.h"
// #include "ApproxMVBB/ComputeApproxMVBB.hpp"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// not good practice
ros::Publisher pub_object;
ros::Publisher pub_plane;
ros::Publisher box_marker;
ros::Publisher box_pose_pub;
ros::Publisher box_target;
double head_tilt = 0;

void update_head_angle(const sensor_msgs::JointStateConstPtr & msg)
{
  // get the head tilt angle not sure if the index is fixed?
  head_tilt = msg->position[5];
}

void callback(const PointCloud::ConstPtr& cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
 
  // filtering
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.5);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);
  
  // Plane extacting ////////////////////////////////////////////////////

  // Optional
  // seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.02);
  double y = 1.0 * cos(head_tilt);
  double z = 1.0 * sin(head_tilt);
  Eigen::Vector3f axis = Eigen::Vector3f(0.0,y,z);
  seg.setAxis(axis);
  seg.setEpsAngle(5.0 * (M_PI / 180.0));
  
  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return;
  }

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  for (std::vector<int>::const_iterator pit = inliers->indices.begin (); pit != inliers->indices.end (); ++pit)
    plane_cloud->push_back ((*cloud_filtered)[*pit]); //*
  
  plane_cloud->width = plane_cloud->size ();
  plane_cloud->height = 1;
  plane_cloud->is_dense = true;

  std::cout << "PointCloud representing the Cluster: " << plane_cloud->size () << " data points." << std::endl;

  // Segment object ////////////////////////////////////////////////////
  pcl::PointIndices::Ptr object_indices (new pcl::PointIndices);
  double z_min = 0.05, z_max = 0.5; // we want the points above the plane, no farther than 5 cm from the surface
  pcl::PointCloud<pcl::PointXYZ>::Ptr hull_points (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::ConvexHull<pcl::PointXYZ> hull;
  // hull.setDimension (2); // not necessarily needed, but we need to check the dimensionality of the output
  hull.setInputCloud(plane_cloud);
  hull.reconstruct(*hull_points);
  if (hull.getDimension () == 2)
  {
    pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
    prism.setInputCloud (cloud_filtered);
    prism.setInputPlanarHull (hull_points);
    prism.setHeightLimits (z_min, z_max);
    prism.segment(*object_indices);
  }
  else
  {
    PCL_ERROR ("The input cloud does not represent a planar surface.\n");
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  for (std::vector<int>::const_iterator pit = object_indices->indices.begin (); pit != object_indices->indices.end (); ++pit)
    object_cloud->push_back ((*cloud_filtered)[*pit]); //*
  object_cloud->width = object_cloud->size ();
  object_cloud->height = 1;
  object_cloud->is_dense = true;

  std::cout << "Object cloud: " << object_cloud->size () << " data points." << std::endl;

  if(object_cloud->size() == 0)
  {
    return;
  }
  
  sensor_msgs::PointCloud2 plane_output;
  pcl::toROSMsg(*plane_cloud, plane_output);
  plane_output.header.frame_id = "head_camera_rgb_optical_frame"; 
  pub_plane.publish(plane_output);

  // Clustering
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclid;
  euclid.setInputCloud(object_cloud);
  euclid.setClusterTolerance(0.02);
  euclid.setMinClusterSize(100);
  euclid.setMaxClusterSize(25000);
  euclid.extract(cluster_indices);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_max (new pcl::PointCloud<pcl::PointXYZ>);
  int max_size = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& idx : it->indices)
      cloud_cluster->push_back ((*object_cloud)[idx]); //*
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    if(cloud_cluster->size() > max_size)
    {
      pcl::copyPointCloud(*cloud_cluster, *cloud_cluster_max);
      max_size = cloud_cluster->size();
      // std::cout << "cluster size = " << cloud_cluster->size() << std::endl;
    }
  }
  if(cloud_cluster_max->size() == 0)
  {
    return;
  }
  
  sensor_msgs::PointCloud2 object_output;
  pcl::toROSMsg(*cloud_cluster_max, object_output);
  object_output.header.frame_id = "head_camera_rgb_optical_frame"; 
  pub_object.publish(object_output);

  // Bounding box
  pcl::PointCloud<pcl::PointXYZRGB> object_cloud_xyzrgb;
  pcl::copyPointCloud(*cloud_cluster_max, object_cloud_xyzrgb);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr extract_out(new pcl::PointCloud<pcl::PointXYZRGB>);
  shape_msgs::SolidPrimitive shape;
  geometry_msgs::Pose box_pose;
  // for(auto myc : coefficients->values)
  // {
  //   std::cout << myc << ", ";
  // }
  // std::cout << std::endl;

  // very bad code
  // fixing the normal vector of the plane segmentation
  if(coefficients->values[3] < 0)
  {
    coefficients->values[0] = -coefficients->values[0];
    coefficients->values[1] = -coefficients->values[1];
    coefficients->values[2] = -coefficients->values[2];
    coefficients->values[3] = -coefficients->values[3];
  }

  simple_grasping::extractShape(object_cloud_xyzrgb, coefficients, *extract_out, shape,
                                box_pose);

  if (shape.type == shape_msgs::SolidPrimitive::BOX) {
    pcl::PointXYZRGB min;
    pcl::PointXYZRGB max;
    pcl::getMinMax3D<pcl::PointXYZRGB>(*extract_out, min, max);

    visualization_msgs::Marker object_marker;
    object_marker.ns = "objects";
    object_marker.id = 0;
    object_marker.header.frame_id = "head_camera_rgb_optical_frame";
    object_marker.type = visualization_msgs::Marker::CUBE;
    object_marker.color.g = 1;
    object_marker.color.a = 0.3;

    object_marker.pose = box_pose;

    object_marker.scale.x = shape.dimensions[0];
    object_marker.scale.y = shape.dimensions[1];
    object_marker.scale.z = shape.dimensions[2];
    box_marker.publish(object_marker);
    box_pose_pub.publish(box_pose);
    robotics_labs::BoxTarget box_target_msg;
    box_target_msg.box_scale = object_marker;
    box_target_msg.box_pose = box_pose;
    box_target.publish(box_target_msg);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  pub_object = nh.advertise<PointCloud> ("points2_object", 1);
  pub_plane = nh.advertise<PointCloud> ("points2_plane", 1);
  box_marker = nh.advertise<visualization_msgs::Marker> ("box_marker", 1);
  box_pose_pub = nh.advertise<geometry_msgs::Pose> ("box_target_pose", 1);
  box_target = nh.advertise<robotics_labs::BoxTarget> ("box_target", 1);
  ros::Subscriber sub = nh.subscribe<PointCloud>("/head_camera/depth_downsample/points", 1, callback);
  ros::Subscriber joint_sub = nh.subscribe<sensor_msgs::JointState>("joint_states", 1, update_head_angle);
  ros::spin();
}