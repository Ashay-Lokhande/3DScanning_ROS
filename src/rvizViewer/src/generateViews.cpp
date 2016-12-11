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
#include <geometry_msgs/PoseStamped.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/publisher.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/tf.h>
#include "findPoints.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using namespace std;
#define PI 3.141592653589793238462643383279502884197169
#define CAMERA_HEIGHT 0;

//count is a varuiable used to make sure that the node only generates the poses once
bool first_time = true;
//A two dimensional vector used to hold the poses generated around the object
//std::vector<std::vector<geometry_msgs::Pose> > pose_2Dcontainer;
std::vector<std::vector<finalFilteredCloud> > all_view_information;

geometry_msgs::Point center;

std::vector<std::vector<finalFilteredCloud> > generateViews(const PointCloud::ConstPtr& msg)
{
    
    // if(first_time)
    // {
    	float numPoints = 0;
    	//printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);

    	/*The radius of the hypothetical circle around teh center of the point cloud where
		  generate our poses										*/

    	float dist_circle_center, max_x, min_x, max_y, min_y;
    	
    	//Finds the center of the PointCloud
    	BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
    	{
    		if(pt.x > max_x)
    			max_x = pt.x;
    		else if(pt.x < min_x)
    			min_x = pt.x;
    		if(pt.y > max_y)
    			max_y = pt.y;
    		else if(pt.y < min_y)
    			min_y = pt.y;

    		center.x += pt.x;
    		center.y += pt.y;
    		center.z += pt.z;
    		numPoints++;
    	}
    		center.x /= numPoints;
    		center.y /= numPoints;
    		center.z /= numPoints;
    		if((max_x - min_x) > (max_y - min_y))
    			dist_circle_center = (max_x - min_x) * 1.5;
    		else
    			dist_circle_center = (max_y - min_y) * 1.5;
    	/*
    	Looping structure to create the poses and load them into a two dimmensional list
					GENERATING THE POSES												
						1) Constant z
						2) Constant difference from center of PointCloud
						3) Stored in a 2D vector
							-- Each row is a different coordinate position of the pose
							-- Each column is a different orientation (angle) of the pose
		*/

        // vector used to collect all the filtered clouds, we will develop most efficienct sets from this list of filetered clouds
        //std::vector<finalFilteredCloud> filteredObjects;

    	//Finds the points of the circle around the center of the object	
    	for (float theta = 0; theta < 2*PI; theta += 0.175){
    		const float buffer_Dist = 2; //this value is used to give a bsae distance of the circle from the object
    		float x = (buffer_Dist + dist_circle_center) * (float) cos(theta);
    		float y = (buffer_Dist+ dist_circle_center) * (float) sin(theta);
    		float yaw =  PI + theta; // (float) sin(.5 * theta);
    		//std::vector<geometry_msgs::Pose> new_coordinate_pose;
             std::vector<finalFilteredCloud> filteredObjects;
    		//for loop for the 5 different orientations of the poses
    		for(float pitch_angle = -1 * (2 * PI) / 6; pitch_angle < ((2 * PI) / 6) + 0.00002; pitch_angle += PI / 6){
    			geometry_msgs::Pose viewPoint;
    			float pitch = PI + /*(PI / 2) + */pitch_angle;// (float) sin(.5 * pitch_angle);
    			viewPoint.position.x = x;
    			viewPoint.position.y = y;
    			viewPoint.position.z = CAMERA_HEIGHT;
    			//viewPoint.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, pitch, yaw);
    			
                // below there is a one to one correspondence between a viewPoint and the filteredObject produced.
                // in other words, the viewPoint at new_coordinate_pose.get(i) produced the filteredCloud at filteredObjects.get(i)

    			//Add the calculated pose to the array of poses
    			//new_coordinate_pose.push_back(viewPoint);

                // filtering out the cloud based on input and collecting all those point clouds
                filteredObjects.push_back(findPoints(viewPoint, msg));
                //printf ("Percent of object viewed is: %f\n", findPoints(viewPoint, msg));
    		}
    		//Add the vector of poses containing the same position but different orientations
    		//pose_2Dcontainer.push_back(new_coordinate_pose);
            all_view_information.push_back(filteredObjects);
    	}

        return all_view_information;

        /*
        printf("IM FINISHED WITH THE FOR LOOP (GENERATEVIEWS)\n");



        // at this point we want to visualize the filteredClouds produced and the viewPoint that generated them.

        // publish the data collected
        // go through the filtered objects and publish them accordingly
        // ERROR HERE!! problem is that a topic must contain only characters from a-z and A-Z, so it is hard to make something unique because number not allowed
        
        // below code for testing purposes
        // refactor to print all the filtered point clouds
        int i = 0;
        int minI = 0;
        int maxI = 0;
        double min = 101;
        double max = -1;
        ros::NodeHandle nh;
        finalFilteredCloud maxViewable; // NULL
        for (std::vector<finalFilteredCloud>::iterator it = filteredObjects.begin() ; it != filteredObjects.end(); ++it){
            if ((*it).percentageViewed > max){
                maxViewable = (*it);
                max = (*it).percentageViewed;
            }
        }

        string publishTopicFilteredCloud = "filteredCloud";
        string publicTopicViewablePoint = "viewedFrom";
        // getting ready to advertise the filtered point cloud
        ros::Publisher cloudPub = nh.advertise<PointCloud> (publishTopicFilteredCloud, 1);

        // getting ready to advertise the view point
        ros::Publisher viewPointPub = nh.advertise<geometry_msgs::PoseStamped> (publicTopicViewablePoint, 1);
        // setting up PoseStamped object to publish
        geometry_msgs::PoseStamped publishedPoint;
        publishedPoint.header.frame_id = "/map";
        publishedPoint.pose = maxViewable.viewedFrom;

        pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud = maxViewable.cloud;

        printf("viewed from (%f, %f, %f) - percentage viewed!: %f\n", 
                maxViewable.viewedFrom.position.x,
                maxViewable.viewedFrom.position.y,
                maxViewable.viewedFrom.position.z,
                maxViewable.percentageViewed);

        first_time = false;
        findPoints(publishedPoint.pose, msg);
        ros::Rate loop_rate(4);
        while (nh.ok()) {
        	// publish it -> the filtered point cloud and where it was a viewed
            cloudPub.publish (filteredCloud);
            viewPointPub.publish(publishedPoint);
       //     printf("The Quaternion is: x = %f, y = %f, z = %f, w = %f \n", 
       //     	publishedPoint.pose.orientation.x, publishedPoint.pose.orientation.y,
       //     	publishedPoint.pose.orientation.z, publishedPoint.pose.orientation.w);
            ros::spinOnce ();
            loop_rate.sleep ();
        }
    }
    */
}

// int main(int argc, char** argv)
// {
//     //Subscriber 
//     ros::init(argc, argv, "generate_views");
//     ros::NodeHandle nh;
//     // ros::Subscriber sub = nh.subscribe<PointCloud>("cloud", 1, callback);
//     ros:: spin();


//     //Publsher
//     ros::NodeHandle n;
//     ros::Publisher pub = n.advertise<std::vector<std::vector<geometry_msgs::Pose> > >("poses", 1);
//     ros::Rate loop_rate(10);

//     while (ros::ok())
//     {
//         pub.publish(pose_2Dcontainer);
//         loop_rate.sleep();
//     }

//     //ros::spin();
//}


