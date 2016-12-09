/*
    Requirements:
        - Multidimensional Boolean Array/Vector for each view
        - A view space (all generated views for the corresponding .pcd)
        - This node to handle to formation of view sets using logical xor / or between the multidimensioanl boolean arrays/vectors


    To do:
    	- findPoints.cpp and generateViews
    		- in generateViews pass to findPoints a pose and expect findPoints to return type pose_object
    			- This object will include the origional pose, the percent of points visible, and the boolean matrix
    			- This pose_object is then pushed onto a two dimensional array of pose_objects (which replaces the pose2Dcontainer)
    		- Implement the formation of a boolean matrix in findPoints.cpp
    	- generateViewSet
    		- call generateViews and expect a 2D array of pose objects returned
    		- generate sets of views based on boolean matrices
    			- create a viewset object which includes
    				- vector of pose objects used
    				- combined boolean matrix 	(or/Xor them together)
    				- combined percent points viewed
    				- cost to move to those views (rotational change from camera is virtually negligible)


*/

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include "findPoints.h"
#include "pose_object.h"
#include "viewSet_object.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef std::vector<std::vector<bool>> booleanMatrix;
typedef std::vector<std::vector<pose_object>> pose_obj_array;
typedef std::std::vector<pose_object> Pose_Set;

void callback(const PointCloud::ConstPtr& msg)
{
	// Calls generate views and expects all pose object info returned
    pose_obj_array views = generate_views();

    //Create Initial Viewsets Based on the starting position
    
    viewSet_object first;
    viewSet_object second;
    viewSet_object third;
    viewSet_object fourth;
    viewSet_object fifth;

    for(std::vector<pose_object> current_array: pose_obj_array;)
    {

    }


    //ROS_INFO("Center x = %f,  y = %f,  z = %f \n", center.x, center.y, center.z);

    /*
    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
        printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
    */
}

int main(int argc, char** argv)
{
    //Subscriber 
    ros::init(argc, argv, "generate_views");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<PointCloud>("cloud", 1, callback);
    ros:: spin();

    //ros::spin();
}
