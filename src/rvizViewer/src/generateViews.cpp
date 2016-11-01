/* 
    This .cpp file will be used to generate the different camera angles around the
    .pcd file. It will pass these views (poses) to the file findPoints.cpp


    Program stucture:
    
    class{
        .
        .
        .
        .
            Generates the views around the .pcd as poses
        .
        .
        .
        for(){
            - goes through the list of poses as passes them over to
                the findPoints.cpp
            - A parrallel array will be made to hold the value this class will
                return

        }
    
        .
        .
        .
        .
        TBD

    }

*/


#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//count is a varuiable used to make sure that the node only generates the poses once
bool first_time = false;
//A two dimensional vector used to hold the poses generated around the object
std::vector<std::vector<geometry_msgs::Pose> > pose_2Dcontainer;

geometry_msgs::Point center;

void callback(const PointCloud::ConstPtr& msg)
{
    
    if(!first_time)
    {
    	int numPoints = 0;
    	printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);

    	/*The radius of the hypothetical circle around teh center of the point cloud where
		  generate our poses															*/
    	int circle_radius = (msg->width * 1.5);
    	
    	//Finds the center of the PointCloud
    	BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
    	{
    		center.x += pt.x;
    		center.y += pt.y;
    		center.z += pt.z;
    		numPoints++;
    	}
    		center.x /= numPoints;
    		center.y /= numPoints;
    		center.z /= numPoints;
    
    	/*
    	Looping structure to create the poses and load them into a two dimmensional list
					GENERATING THE POSES												
						1) Constant z
						2) Constant difference from center of PointCloud
						3) Stored in a 2D vector
							-- Each row is a different coordinate position of the pose
							-- Each column is a different orientation (angle) of the pose
		*/


    	first_time = true;
    }
    
    
    ROS_INFO("Center x = %f,  y = %f,  z = %f \n", center.x, center.y, center.z);

    /*
    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
        printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
    */
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "generate_views");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<PointCloud>("cloud", 1, callback);
    ros::spin();
}


