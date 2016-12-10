/*
    Requirements:
        - Multidimensional Boolean Array/Vector for each view
        - A view space (all generated views for the corresponding .pcd)
        - This node to handle to formation of view sets using logical xor / or between the multidimensioanl boolean arrays/vectors

    What needs to be made and were:
        - Generate Views
            - Find a way to publish or pass the Views generated in this file tp generateViewSet.cpp
        - Find Points
            - Find a method to either implement in the current findPoints.cpp or create a new .cpp
                in order to create and pass to generateViewSet.cpp a multidimensional boolean
                array/vector
        - Generate View Set
            - Make a method of forming view sets through the use of these boolean sets
                - Using logical OR/XOR we can combine the views and fijnd a way of comparing viewsets
*/

 #include <ros/ros.h>
 #include <pcl_ros/point_cloud.h>
 #include <pcl/point_types.h>
 #include <boost/foreach.hpp>
 #include <ros/package.h>
 #include <geometry_msgs/Pose.h>
 #include <geometry_msgs/Point.h>
 #include "findPoints.h"
 #include "generateViews.h"
 // #include "ViewSet_Object.h"



 // typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
 // typedef std::vector<std::vector<bool>> booleanMatrix;
 // typedef std::vector<std::vector<pose_object>> pose_obj_array;
 // typedef std::std::vector<pose_object> Pose_Set;

 void callback(const PointCloud::ConstPtr& msg)
 {
     // Calls generate views and expects all pose object info returned
     // pose_obj_array views = generate_views();

     // //Create Initial Viewsets Based on the starting position
    
     // viewSet_object first;
     // viewSet_object second;
     // viewSet_object third;
     // viewSet_object fourth;
     // viewSet_object fifth;

     // for(std::vector<pose_object> current_array: pose_obj_array;)
     // {

     // }
     // ROS_INFO("Center x = %f,  y = %f,  z = %f \n", center.x, center.y, center.z);

     /*
     BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
         printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
     */
 }

 int main(int argc, char **argv)
 {
     //Subscriber 
     ros::init(argc, argv, "generate_views");
     ros::NodeHandle nh;
     ros::Subscriber sub = nh.subscribe<PointCloud>("cloud", 1, callback);
     ros:: spin();

     //ros::spin();
 }
