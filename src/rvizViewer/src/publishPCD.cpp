/*
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <ros/package.h>
#include <ros/publisher.h>
#include <pcl_ros/publisher.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv) {
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<PointCloud> ("points2", 1);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("spray_bottle.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file spray_bottle.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from spray_bottle.pcd with the following fields: "
            << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cout << "    " << cloud->points[i].x
              << " "    << cloud->points[i].y
              << " "    << cloud->points[i].z << std::endl;
 
  ros::Rate loop_rate(4);
  while (nh.ok())
  {
    cloud->header.stamp = ros::Time::now().toNSec();
    pub.publish (cloud);
    ros::spinOnce ();
    loop_rate.sleep ();
  }
}
*/

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/io/pcd_io.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<PointCloud> ("cloud", 1);

  PointCloud::Ptr cloud (new PointCloud);

   if (pcl::io::loadPCDFile<pcl::PointXYZ> ("spray_bottle.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file spray_bottle.pcd \n");
      ROS_INFO("It is not publishing");
    return (-1);
  }

  ROS_INFO("It is publishing");

  ros::Rate loop_rate(4);
  while (nh.ok())
  {
    cloud->header.stamp = ros::Time::now().toNSec();
    pub.publish (*cloud);
    ros::spinOnce ();
    loop_rate.sleep ();
  }
}
