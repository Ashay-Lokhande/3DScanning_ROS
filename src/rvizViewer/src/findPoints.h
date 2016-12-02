#ifndef FINDPOINTS_H
#define FINDPOINTS_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <math.h>
#include <geometry_msgs/Pose.h>

struct finalFilteredCloud {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    geometry_msgs::Pose viewedFrom;
    double percentageViewed;

};

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

finalFilteredCloud findPoints(const geometry_msgs::Pose, const PointCloud::ConstPtr&);
#endif /* FINDPOINTS_H */