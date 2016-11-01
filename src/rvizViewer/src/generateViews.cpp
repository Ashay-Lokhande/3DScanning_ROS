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

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//count is a varuiable used to make sure that the node only generates the poses once
bool first_time = false;
//A two dimensional vector used to hold the poses generated around the object
std::vector<std::vector<geometry_msgs::Pose> > pose_2Dcontainer;

void callback(const PointCloud::ConstPtr& msg)
{
    printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    
    if(!first_time){
    	int circle_radius = (msg->width * 1.5) +
    	//Looping structure to create the poses and load them into a two dimmensional list
    }
    
    
    


    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
        printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);

    first_time = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "generate_views");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<PointCloud>("cloud", 1, callback);
    ros::spin();
}


