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
#include <fetch_delivery_system/BoxTarget.h>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "visualization_msgs/Marker.h"
#include "simple_grasping/shape_extraction.h"
#include "shape_msgs/SolidPrimitive.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// not good practice, might want to get rid of globals in future
ros::Publisher pub_object;
ros::Publisher pub_plane;
ros::Publisher box_marker;
ros::Publisher box_pose_pub;
ros::Publisher box_target;
double head_tilt = 0;

void update_head_angle(const sensor_msgs::JointStateConstPtr &msg)
{
  // get the head tilt angle not sure if the index is fixed?
  head_tilt = msg->position[5];
}

void callback(const PointCloud::ConstPtr &cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  // filtering
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.5);
  //pass.setFilterLimitsNegative (true);
  pass.filter(*cloud_filtered);

  // Plane extraction
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // seg.setOptimizeCoefficients (true); // optional, refines estimated plane coeffecients, takes more time
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.02);
  double y = 1.0 * cos(head_tilt);
  double z = 1.0 * sin(head_tilt);
  Eigen::Vector3f axis = Eigen::Vector3f(0.0, y, z);
  seg.setAxis(axis);
  seg.setEpsAngle(5.0 * (M_PI / 180.0));

  seg.setInputCloud(cloud_filtered);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0)
  {
    PCL_ERROR("Could not estimate a planar model for the given dataset.");
    return;
  }

  std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::copyPointCloud(*cloud_filtered, *inliers, *plane_cloud);

  std::cout << "PointCloud representing the Cluster: " << plane_cloud->size() << " data points." << std::endl;

  // publish the plane cloud as a ROS message
  sensor_msgs::PointCloud2 plane_output;
  pcl::toROSMsg(*plane_cloud, plane_output);
  plane_output.header.frame_id = "head_camera_rgb_optical_frame";
  pub_plane.publish(plane_output);

  // Segment object
  pcl::PointIndices::Ptr object_indices(new pcl::PointIndices);
  double z_min = 0.05, z_max = 0.5; // we want the points above the plane, between 5 cm and 50 cm from the surface
  pcl::PointCloud<pcl::PointXYZ>::Ptr hull_points(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::ConvexHull<pcl::PointXYZ> hull;
  // hull.setDimension (2); // not necessarily needed, but we need to check the dimensionality of the output
  hull.setInputCloud(plane_cloud);
  hull.reconstruct(*hull_points);
  if (hull.getDimension() == 2)
  {
    pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
    prism.setInputCloud(cloud_filtered);
    prism.setInputPlanarHull(hull_points);
    prism.setHeightLimits(z_min, z_max);
    prism.segment(*object_indices);
  }
  else
  {
    PCL_ERROR("The input cloud does not represent a planar surface.\n");
  }

  // a point cloud of possible objects
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::copyPointCloud(*cloud_filtered, *object_indices, *object_cloud);

  std::cout << "Object cloud: " << object_cloud->size() << " data points." << std::endl;

  if (object_cloud->size() == 0)
  {
    return;
  }

  // Clustering
  // find the clusters
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclid;
  euclid.setInputCloud(object_cloud);
  euclid.setClusterTolerance(0.02);
  euclid.setMinClusterSize(100);
  euclid.setMaxClusterSize(25000);
  euclid.extract(cluster_indices);

  // find the closest cluster
  pcl::PointCloud<pcl::PointXYZ>::Ptr nearest_cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  double shortest_distance = INFINITY;
  double current_distance;
  Eigen::Vector4d centroid4;
  Eigen::Vector3d centroid3;
  pcl::PointIndices nearest_cluster_indices;
  for (pcl::PointIndices it : cluster_indices)
  {
    // find centroid of cluster
    pcl::compute3DCentroid(*object_cloud, it, centroid4);
    // std::cerr << "centroid: " << centroid4 << std::endl;
    // compute distance from centroid to head camera
    centroid3 = centroid4.head<3>();
    current_distance = centroid3.norm();
    if (current_distance < shortest_distance)
    {
      // reset shortest distance
      shortest_distance = current_distance;
      // store nearest cluster indicies
      nearest_cluster_indices = it;
    }
    
  }
  if (nearest_cluster_indices.indices.size() == 0)
  {
    return;
  }
  else
  {
    // create nearest cluster cloud
    pcl::copyPointCloud(*object_cloud, nearest_cluster_indices, *nearest_cluster_cloud);
    std::cerr << "distance: " << shortest_distance << std::endl;
  }

  sensor_msgs::PointCloud2 object_output;
  pcl::toROSMsg(*nearest_cluster_cloud, object_output);
  object_output.header.frame_id = "head_camera_rgb_optical_frame";
  pub_object.publish(object_output);

  // Bounding box
  pcl::PointCloud<pcl::PointXYZRGB> object_cloud_xyzrgb;
  pcl::copyPointCloud(*nearest_cluster_cloud, object_cloud_xyzrgb);
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
  if (coefficients->values[3] < 0)
  {
    coefficients->values[0] = -coefficients->values[0];
    coefficients->values[1] = -coefficients->values[1];
    coefficients->values[2] = -coefficients->values[2];
    coefficients->values[3] = -coefficients->values[3];
  }

  simple_grasping::extractShape(object_cloud_xyzrgb, coefficients, *extract_out, shape,
                                box_pose);

  if (shape.type == shape_msgs::SolidPrimitive::BOX)
  {
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
    fetch_delivery_system::BoxTarget box_target_msg;
    box_target_msg.box_scale = object_marker;
    box_target_msg.box_pose = box_pose;
    box_target.publish(box_target_msg);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  pub_object = nh.advertise<PointCloud>("points2_object", 1);
  pub_plane = nh.advertise<PointCloud>("points2_plane", 1);
  box_marker = nh.advertise<visualization_msgs::Marker>("box_marker", 1);
  box_pose_pub = nh.advertise<geometry_msgs::Pose>("box_target_pose", 1);
  box_target = nh.advertise<fetch_delivery_system::BoxTarget>("box_target", 1);
  ros::Subscriber sub = nh.subscribe<PointCloud>("/head_camera/depth_downsample/points", 1, callback);
  ros::Subscriber joint_sub = nh.subscribe<sensor_msgs::JointState>("joint_states", 1, update_head_angle);
  ros::spin();
}