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

void callback(const PointCloud::ConstPtr& msg)
{
    printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
        printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "generate_views");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<PointCloud>("cloud", 1, callback);
    ros::spin();
}


