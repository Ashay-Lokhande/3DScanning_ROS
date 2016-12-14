#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <math.h>
#include <geometry_msgs/Pose.h>
#include <set>
#include <map>


// consists of redundancy, remove later
// copied from rvizView to work with PointCloud2 object
#include <ros/ros.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/publisher.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/publisher.h>
#include <string>
#include <pcl_ros/point_cloud.h>

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
#define PI 3.141592653589793238462643383279502884197169

// This angle is equal to the range(in radians) that the camera can see vertcally up and down
#define camera_pitch_angle_range PI / 3
// This angle is equal to the range(in radians) that the camera can see horiziontally
#define camera_yaw_angle_range PI / 3

// struct that represnts a point in the point cloud
// it may be viewed (1) or not (0)
struct point_viewed{
    geometry_msgs::Pose point;
    int index;
    int is_viewed;
};

// struct to represent information of the filtered cloud
// consists of filtered cloud and other useful information
// to add more useful features change it below and in the .h file
struct finalFilteredCloud {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    geometry_msgs::Pose viewedFrom;
    double percentageViewed;
    std::map<int, point_viewed> point_in_cloud;
};



// calculating slope of line formed by two points in 3d space
float calculate_slope(float pt_x, float pt_y, float pt_z, float pt2_x, float pt2_y, float pt2_z)
{
    float rise = pt2_z - pt_z;

    float square = pow((pt2_x - pt_x), 2) + pow((pt2_y - pt_y), 2);
    float run = sqrt(square);
    return rise/run;
}   

// calculate the slope in a 2d plane - used to calculate the slope in the y-z plane
float calculate_2d_slope (float x, float y, float pt_x, float pt_y)
{
    float rise = pt_y - y;
    float run = pt_x - x;
    return rise/run;
}

// calculate the angle from the horizontal for two points in a 2d plane - used to calculate the angle in the x-z plane
float calculate_angle (float x, float z, float pt_x, float pt_z)
{
    float angle = atan2(pt_z - z, pt_x - x);
    return angle;
}

// calculating distance between two points in  3d space
float distance(float x1, float y1, float z1, float x2, float y2, float z2){

    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
}


float getPitch(const geometry_msgs::Quaternion qt) {
  return atan2(2*(qt.y*qt.z + qt.w*qt.x), qt.w*qt.w - qt.x*qt.x - qt.y*qt.y + qt.z*qt.z);
 //return atan2(2*(qt.x * qt.w - qt.y * qt.z), 1 - 2*(qt.x * qt.x - qt.z * qt.z));
}

float getYaw(const geometry_msgs::Quaternion qt){
  return asin(-2*(qt.x*qt.z - qt.w*qt.y));
}

float getRoll(const geometry_msgs::Quaternion qt) {
  return atan2(2*(qt.x*qt.y + qt.w*qt.z), qt.w*qt.w + qt.x*qt.x - qt.y*qt.y - qt.z*qt.z);
}

float find_angle(float x1, float y1, float z1, float x2, float y2, float z2){


    float num = (x1 * x2 + y1 * y2 + z1 * z2);
    float denom = sqrt(pow(x1, 2) + pow(y1, 2) + pow(z1, 2)) * sqrt(pow(x2, 2) + pow(y2, 2) + pow(z2, 2));

    return acos(num / denom);
}


// method to find viewable points from a given view. 
// arg1: Passed in a pose that represents a point in space
// arg2: the point cloud object of the pcd file
finalFilteredCloud findPoints(const geometry_msgs::Pose createdPoint, const PointCloud::ConstPtr& msg)
{

    // the final object to return
    finalFilteredCloud ret;
    // printing the current view point

    // x,y,z represents the cooridnates of the view 
    float x = (createdPoint.position.x) ;// / (float) 250.0;
    float y = (createdPoint.position.y) ;// / (float) 250.0;
    float z = (createdPoint.position.z) ;// / (float) 250.0;
    ret.viewedFrom.position.x = x;
    ret.viewedFrom.position.y = y;
    ret.viewedFrom.position.z = z;
    ret.viewedFrom.orientation.x = createdPoint.orientation.x;
    ret.viewedFrom.orientation.y = createdPoint.orientation.y;
    ret.viewedFrom.orientation.z = createdPoint.orientation.z;
    ret.viewedFrom.orientation.w = createdPoint.orientation.w;  

    // pitch_angle and theta represent the angles relative to the view
    // pitch_angle is the rotation of the camera
    // theta is the rotation of the robot relative to the center

    float pitch = getPitch(createdPoint.orientation);
    float yaw = getYaw(createdPoint.orientation);

    // figure out the max slope and min slope on the y, z plane
    // given a specific camera pitch angle - this will help limit which points should be considered viewable
    // assuming a 60 degree camera viewfinder (vertically)
    float max_slope, min_slope;

    max_slope = pitch + (camera_pitch_angle_range / 2); //tan(pitch + (camera_pitch_angle_range / 2));
    min_slope = pitch - (camera_pitch_angle_range / 2); //tan(pitch - (camera_pitch_angle_range / 2));

    // total number of points in the point cloud
    int size = 0;

    // map that will be assigned to finalFilteredCloud when returned
    std::map<int, point_viewed> point_in_cloud;

    // idea is that for a given line from the view point, slope is the same
    // we want to count for a single point and exclude all the other points on that line
    // by that way we count for a single point and not count points behind the counted 
    // point
    // if slope is already contained, then check which point is closer
    // to the view point
    // map for: slope -> point
    std::map<float, point_viewed> viewablePoints;
    std::map<float, point_viewed>::iterator it;
    int keyIndex = 0;
    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points) {
        // ensures that the point in question is within the camera viewpoint (using the physical slopes calculated above)
        // I am assuming that vertical slope is a calculation of the y and z coordinates
        float vertical_slope = calculate_angle(y, z, pt.y, pt.z);//calculate_slope(x, y, z, pt.x, pt.y, pt.z);

        //if (vertical_slope >= min_slope && vertical_slope <= max_slope) {
            // calculate the angle between the camera and the point in question
            double angle = calculate_angle(x, y, pt.x, pt.y);
            // if it is within the horizontal view range of the camera, proceed, otherwise stop
            //if (angle <= (yaw / 2) + camera_yaw_angle_range && 
            //        angle >= (yaw / 2) - camera_yaw_angle_range
                    //abs(angle) <= yaw / 2
            //    ){

                float slope_pt_to_view = round(calculate_slope(x, y, z, pt.x, pt.y, pt.z) * 100000.0) / 100000.0 ;
                it = viewablePoints.find(slope_pt_to_view);

                // if slope is already present
                if(it != viewablePoints.end()) {



                    // if the slop is already taken into account
                    // then we must see the point it corresponds to is the closest to the view point
                    // if this new point is closer then must updates the map accordingly
                    geometry_msgs::Pose existingPoint = it->second.point; // get the point
                    int indexOfExistingPoint = it->second.index;
                    float oldDistance = distance(x, y, z, existingPoint.position.x, existingPoint.position.y, existingPoint.position.z);
                    float newDistance = distance(x, y, z, pt.x, pt.y, pt.z);
                    if(newDistance < oldDistance) {
                        std::map<int, point_viewed>::iterator isPresent = point_in_cloud.find(indexOfExistingPoint); // no get method in map in C++, returns an iterator
                        isPresent->second.is_viewed = 0; // older point is not viewable now. replace it with a closer point which is viewble

                    	geometry_msgs::Pose pose;
            			pose.position.x = pt.x;
            			pose.position.y = pt.y;
            			pose.position.z = pt.z;

                        point_viewed newPoint;
                        newPoint.point = pose;
                        newPoint.is_viewed = 1;
                        newPoint.index = keyIndex;


                        viewablePoints.insert(std::pair<float, point_viewed> (slope_pt_to_view, newPoint));
                        point_in_cloud.insert(std::pair<int, point_viewed> (keyIndex, newPoint));
                        //printf("Old point: %f, %f, %f with distance of %f\n", existingPoint.x, existingPoint.y, existingPoint.z, oldDistance);
                        //printf("New point: %f, %f, %f with distance of %f\n", newPoint.x, newPoint.y, newPoint.z, newDistance);
                    }

                } else {
                    // first time looking at this point
                    // just add it
                    geometry_msgs::Pose pose;
                    pose.position.x = pt.x;
                    pose.position.y = pt.y;
                    pose.position.z = pt.z;

                    point_viewed newPoint;
                    newPoint.point = pose;
                    newPoint.index = keyIndex;
                    newPoint.is_viewed = 1;
                    viewablePoints.insert(std::pair<float, point_viewed> (slope_pt_to_view, newPoint));
                    point_in_cloud.insert(std::pair<int, point_viewed> (keyIndex, newPoint));
                }
            //}
        //}
        keyIndex++;
        size++; // totalNum points

    }


    // displaying contents of the map
    // multipying by 10000 because %f truncates values, so it appears as if duplictes exist
    // for(it = viewablePoints.begin(); it != viewablePoints.end(); it++){
    //     printf("key: %f, Value: %f, %f, %f\n", it->first * 10000, it->second.x, it->second.y, it->second.z);
    // }


    // at this point we have filtered points based on slope,distance and angle
    // these points need to be used to create a pointcloud2 object
    // once this is created, we will publish this to rviz and view it accoringly
    // references to publishing to rviz check rvizView and generateViews.cpp

    // in progress
    // refer: http://wiki.ros.org/pcl_ros
    // refer: http://pointclouds.org/documentation/tutorials/passthrough.php

    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZ>);

    filteredCloud->width = viewablePoints.size(); // need to verify
    filteredCloud->height = 1;
    filteredCloud->points.resize(filteredCloud->width * filteredCloud->height); 
    filteredCloud->header.frame_id = "/map";
    int filterIndex = 0;

    // loading a new pointcloud object from the points that we collected
    for(it = viewablePoints.begin(); it != viewablePoints.end(); it++, filterIndex++){

    	// ideally no segfaults because filterdCloud size 
    	// is viewable points * 1 == viewablePoints.size();
    	filteredCloud->points[filterIndex].x = it->second.point.position.x;
    	filteredCloud->points[filterIndex].y = it->second.point.position.y;
    	filteredCloud->points[filterIndex].z = it->second.point.position.z;

    }

    // return a new object that represents the cloud to be published
    // along with other information that may be useful
    // to add more useful features, change the struct above and the .h file
    
    ret.cloud = filteredCloud;
    ret.point_in_cloud = point_in_cloud;
    ret.percentageViewed = 100 * viewablePoints.size() / (float) size;
    return ret;
}

