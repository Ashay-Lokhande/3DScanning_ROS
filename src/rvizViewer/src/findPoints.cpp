/*
   This .cpp file will be used to find all the points in the published .pcd object 
   model that are viewable from the pose passed in as a parameter

Attributes:

- Has no main
- filled with functions that are mainly private. The only public method will
be a method called from generate views and returns our quantification value
that indicates the percent of points on the object that are seen by that view

To do:
- need to subscirbe to the topic we are pushing the .pcd pointcloud to in
the file rvizView

 */

#include <ros/ros.h>                                                                
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/publisher.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/publisher.h>
#include <string>
//Used to include poses = views
#include <geometry_msgs/Pose.msg>


class findPoints(){

    //Public function to get the pose from generateViews.cpp
    public void quantifyDataInView(const  geometry_msgs::Pose &current_view)_{

        //call functions
        makeCone(*current_view);
        //retirn vaue
    }

    private [type of cone] makeCone(pose){

        //returns generated cone stucture
    }

}
