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

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


// represnts a point with x,y,z coordinates
struct Point {

    float x;
    float y;
    float z;
};


class PMDCloudPublisher
{
    // protected:
    //     std::string tf_frame;
    //     ros::NodeHandle nh;
    //     ros::NodeHandle private_nh;

    // public:
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    //     std::string file_name, cloud_topic;
    //     pcl_ros::Publisher<sensor_msgs::PointCloud2> pub;

    //     PMDCloudPublisher()
    //         : tf_frame("/map"),
    //         private_nh("~")
    // {
    //     cloud_topic = "cloud";
    //     pub.advertise(nh, cloud_topic.c_str(), 1); // published here
    //     private_nh.param("frame_id", tf_frame, std::string("/map"));
    //     ROS_INFO_STREAM("Publishing data on topic \"" << nh.resolveName(cloud_topic) << "\" with frame_id \"" << tf_frame << "\"");
    // }

    //     bool spin ()
    //     {
    //         int nr_points = cloud->width * cloud->height;
    //         std::string fields_list = pcl::getFieldsList(cloud);
    //         ros::Rate r(40);

    //         while(nh.ok ())
    //         {
    //             ROS_DEBUG_STREAM_ONCE("Publishing data with " << nr_points
    //                     << " points " << fields_list
    //                     << " on topic \"" << nh.resolveName(cloud_topic)
    //                     << "\" in frame \"" << cloud.header.frame_id << "\"");
    //             // cloud.header.stamp = ros::Time::now();
    //             pub.publish(cloud);

    //             r.sleep();
    //         }
    //         return (true);
    //     }
};


void publishData(pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud, std::map<float, geometry_msgs::Pose> viewablePoints){

    // //ros::init (1, temp, "cloud_publisher");
    // ros::NodeHandle nh;

    
    // PMDCloudPublisher c;
    // c.file_name = "spray_bottle.pcd";
    // c.cloud = filteredCloud;
    // nh.getParam(filteredCloud, c.cloud);

    // if (c.start () == -1)
    // {
    //     ROS_ERROR_STREAM("Could not load file \"" << c.file_name  <<"\". Exiting...");
    // }
    // ROS_INFO_STREAM("Loaded a point cloud with " << c.cloud.width * c.cloud.height
    //         << " points (total size is " << c.cloud.data.size() << ") and the following channels: " << pcl::getFieldsList (c.cloud));
    // c.spin();
}


// calculating slope of line formed by two points in 3d space
float calculate_slope(float pt_x, float pt_y, float pt_z, float pt2_x, float pt2_y, float pt2_z)
{
    float rise = pt2_z - pt_z;

    float square = pow((pt2_x - pt_x), 2) + pow((pt2_y - pt_y), 2);
    float run = sqrt(square);
    return rise/run;
}   

float calculate_2d_slope (float x, float y, float pt_x, float pt_y)
{
    float rise = pt_y - y;
    float run = pt_x - x;
    return rise/run;
}

// bool onLine(float view_x, float view_y, float view_z, float pt_x, float pt_y, float pt_z, float check_pt_x, float check_pt_y, float check_pt_z)
// {
//     // Calculate the slope from the view to the check_pt
//     float slope_pt_to_view = calculate_slope(view_x, view_y, view_z, check_pt_x, check_pt_y, check_pt_z);

//     // printf("Slope from view to check_pt: %f\n", slope_pt_to_view);

//     // Calculate slope from the check_pt to the pt
//     float slope_check_pt_to_pt = calculate_slope(view_x, view_y, view_z, pt_x, pt_y, pt_z);

//     // printf("Slope from view to pt: %f\n", slope_check_pt_to_pt);
//     // printf("Are they equivalent?: %d\n", temp);

//     float delta = 0.0000000045f;

//     float diff = fabs(slope_pt_to_view - slope_check_pt_to_pt);

//     bool temp = (diff < delta);
//     // printf("Difference: %f\n", diff);
//     // printf("Boolean: %d\n", temp);

//     //printf("Difference: %f\n", diff);
//     // If they are equal return true
//     return temp;
// }

// calculating distance between two points in  3d space
float distance(float x1, float y1, float z1, float x2, float y2, float z2){

    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
}


// method to find viewable points from a given view. 
// arg1: Passed in a pose that represents a point in space
// arg2: the point cloud object of the pcd file

// PENDING: consider orientation (various angles for a given point) when viewing the object
void findPoints(const geometry_msgs::Pose createdPoint, const PointCloud::ConstPtr& msg)
{


    // printing the current view point
    printf("Coordinates: %f, %f, %f\n", createdPoint.position.x, createdPoint.position.y, createdPoint.position.z);

    // x,y,z represents the cooridnates of the view 
    float x = createdPoint.position.x;
    float y = createdPoint.position.y;
    float z = createdPoint.position.z;

    // pitch_angle and theta represent the angles relative to the view
    // pitch_angle is the rotation of the camera
    // theta is the rotation of the robot relative to the center
    float pitch_angle = (float) (2 * asin(createdPoint.orientation.y));
    float theta = (float) (2 * asin(createdPoint.orientation.z));

    // figure out the max slope and min slope on the y, z plane
    // given a specific camera pitch angle - this will help limit which points should be considered viewable
    // assuming a 60 degree camera viewfinder (vertically)
    float max_y_slope, min_y_slope;

    if (pitch_angle = 30) {
        max_y_slope = 1.73;
        min_y_slope = 0;
    } else if (pitch_angle = 60) {
        max_y_slope = 9;
        min_y_slope = 0.577;
    } else if (pitch_angle = 90) {
        max_y_slope = 1.73;
        min_y_slope = -1.73;
    } else if (pitch_angle = 120) {
        max_y_slope = 9;
        min_y_slope = -0.577;
    } else if (pitch_angle = 150) {
        max_y_slope = 0;
        min_y_slope = -1.73;
    }

    // total number of points in the point cloud
    int size = 0;

    // idea is that for a given line from the view point, slope is the same
    // we want to count for a single point and exclude all the other points on that line
    // by that way we count for a single point and not count points behind the counted 
    // point
    // if slope is already contained, then check which point is closer
    // to the view point
    // map for: slope -> point
    std::map<float, geometry_msgs::Pose> viewablePoints;
    std::map<float, geometry_msgs::Pose>::iterator it;
    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points) {
        // ensures that the point in question is within the camera viewpoint (using the physical slopes calculated above)
        // I am assuming that vertical slope is a calculation of the y and z coordinates
        float vertical_slope = calculate_2d_slope(y, z, pt.y, pt.z);
        if (vertical_slope >= min_y_slope && vertical_slope <= max_y_slope) {
            float slope_pt_to_view = round(calculate_slope(x, y, z, pt.x, pt.y, pt.z) * 100000.0) / 100000.0 ;
            it = viewablePoints.find(slope_pt_to_view);

            // if slope is already present
            if(it != viewablePoints.end()){
                // if the slop is already taken into account
                // then we must see the point it corresponds to is the closest to the view point
                // if this new point is closer then must updates the map accordingly
                geometry_msgs::Pose existingPoint = it->second; // get the point
                float oldDistance = distance(x, y, z, existingPoint.position.x, existingPoint.position.y, existingPoint.position.z);
                float newDistance = distance(x, y, z, pt.x, pt.y, pt.z);
                if(newDistance < oldDistance) {
                	geometry_msgs::Pose newPose;
        			newPose.position.x = pt.x;
        			newPose.position.y = pt.y;
        			newPose.position.z = pt.z;
                    viewablePoints.insert(std::pair<float, geometry_msgs::Pose> (slope_pt_to_view, newPose));
                    //printf("Old point: %f, %f, %f with distance of %f\n", existingPoint.x, existingPoint.y, existingPoint.z, oldDistance);
                    //printf("New point: %f, %f, %f with distance of %f\n", newPoint.x, newPoint.y, newPoint.z, newDistance);
                }

            } else {
                // first time looking at this point
                // just add it
                geometry_msgs::Pose newPose;
                newPose.position.x = pt.x;
                newPose.position.y = pt.y;
                newPose.position.z = pt.z;
                viewablePoints.insert(std::pair<float, geometry_msgs::Pose> (slope_pt_to_view, newPose));
            }
        }
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

    // publishing data set
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<PointCloud> ("fileredCloud", 1);

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
    	filteredCloud->points[filterIndex].x = it->second.position.x;
    	filteredCloud->points[filterIndex].y = it->second.position.y;
    	filteredCloud->points[filterIndex].z = it->second.position.z;

    }

    // publishing to rviz topic
    // currently infinite
    ros::Rate loop_rate(4);
    while (nh.ok()) {
        filteredCloud->header.stamp = ros::Time::now().toNSec();
        pub.publish (filteredCloud);

        printf("view coordinates: %f, %f, %f - viewable: %f\n", x,y,z, 100 * viewablePoints.size() / (float) (size));
        ros::spinOnce ();
        loop_rate.sleep ();
    }

    // for modularity, break into separate methods
    // publishData(filteredCloud, viewablePoints);




    //float viewableAmount = 
    //return viewableAmount; // percentag of points viewed




    // old implementation!

    // traverse through points of the point cloud
    // for a given point we scan all the same points
    // for each pair of points, we check if they have the same slope or comparable slopes
    // if they are similar we discount this because we have seen this point before
    // by this way we count all the points, without duplicates
    // by doing numberInLine <=1 we make sure we count only unique points and not duplicates
    // BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points) {
    //     int numberInLine = 0;
    //     BOOST_FOREACH (const pcl::PointXYZ& pt2, msg->points) {
    //         bool onTheLine = onLine(x, y, z, pt.x, pt.y, pt.z, pt2.x, pt2.y, pt2.z);
    //         if (onTheLine) {
    //             numberInLine++;
    //         }
    //     }
    //     // printf("inView: %d\n", inView);
    //     if (numberInLine <= 1) {
    //         count++;
    //     }
    //     size++;
    // }

    // // printf("Count: %d\n", count);
    // // printf("Size: %d\n", size);

    // //printf("Percent of points viewed from pose: %f\n", 100 * count/(float)size);

    // // finally return a ratio of the points seen to the total number of points 
    // // in the pcl 
    // return (100 * count / (float) (size));
}

// void callback(const PointCloud::ConstPtr& msg)
// {
//     // printf("Percent of points viewed: %f\n", findPoints(msg));
// }

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "generate_views");
//     ros::NodeHandle nh;
//     ros::Subscriber sub = nh.subscribe<PointCloud>("cloud", 1, callback);
//     ros::spin();
// }