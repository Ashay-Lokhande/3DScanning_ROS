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
#include "findPoints.h"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/publisher.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using namespace std;
#define PI 3.141592653589793238462643383279502884197169
#define CAMERA_HEIGHT 0;

//count is a varuiable used to make sure that the node only generates the poses once
bool first_time = false;
//A two dimensional vector used to hold the poses generated around the object
std::vector<std::vector<geometry_msgs::Pose> > pose_2Dcontainer;

geometry_msgs::Point center;

void callback(const PointCloud::ConstPtr& msg)
{
    
    if(!first_time)
    {
    	float numPoints = 0;
    	//printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);

    	/*The radius of the hypothetical circle around teh center of the point cloud where
		  generate our poses															*/
    	float circle_radius = (msg->width * 1.5);
    	
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

        // vector used to collect all the filtered clouds, we will develop most efficienct sets from this list of filetered clouds
        std::vector<finalFilteredCloud> filteredObjects;

    	//Finds the points of the circle around the center of the object	
    	for (float theta = 0; theta < 2*PI; theta += 0.175){
    		float x = circle_radius * (float) cos(theta);
    		float y = circle_radius * (float) sin(theta);
    		std::vector<geometry_msgs::Pose> new_coordinate_pose;
    		//for loop for the 5 different orientations of the poses
    		for(float pitch_angle = 30; pitch_angle <= 150; pitch_angle += 30){
    			geometry_msgs::Pose viewPoint;
    			viewPoint.position.x = x;
    			viewPoint.position.y = y;
    			viewPoint.position.z = CAMERA_HEIGHT;
    			viewPoint.orientation.w = 0;
    			viewPoint.orientation.x = 0;
    			viewPoint.orientation.y = (float) sin(.5 * pitch_angle);
    			viewPoint.orientation.z = (float) sin(.5 * theta); //replace with degree change to look at the center

                // below there is a one to one correspondence between a viewPoint and the filteredObject produced.
                // in other words, the viewPoint at new_coordinate_pose.get(i) produced the filteredCloud at filteredObjects.get(i)

    			//Add the calculated pose to the array of poses
    			new_coordinate_pose.push_back(viewPoint);

                // filtering out the cloud based on input and collecting all those point clouds
                filteredObjects.push_back(findPoints(viewPoint, msg));

                //printf ("Percent of object viewed is: %f\n", findPoints(viewPoint, msg));
    		}
    		//Add the vector of poses containing the same position but different orientations
    		pose_2Dcontainer.push_back(new_coordinate_pose);
    	}


        // at this point we want to visualize the filteredClouds produced and the viewPoint that generated them.

        // publish the data collected
        // go through the filtered objects and publish them accordingly
        // ERROR HERE!! problem is that a topic must contain only characters from a-z and A-Z, so it is hard to make something unique because number not allowed
        
        // below code for testing purposes
        // refactor to print all the filtered point clouds
        int i = 0;
        int minI = 0;
        int maxI = 0;
        float min = 101;
        float max = -1;
        ros::NodeHandle nh;
        for (std::vector<finalFilteredCloud>::iterator it = filteredObjects.begin() ; it != filteredObjects.end(); ++it){
            if((*it).percentageViewed > max){
                min = (*it).percentageViewed;
                minI = i;
                maxI = i;

            }
            i++;
            //printf("Viewed from %f, %f, %f and Percentage %f\n", (*it).viewedFrom.position.x, (*it).viewedFrom.position.y, (*it).viewedFrom.position.z, (*it).percentageViewed);
        }

            string publishTopic = "filteredCloud";
            ros::Publisher pub = nh.advertise<PointCloud> (publishTopic, 1);
            pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud = filteredObjects.at(maxI).cloud; 
            //= (*it).cloud;
            printf("percentage viewed!!!!!: %f\n", filteredObjects.at(maxI).percentageViewed);
            ros::Rate loop_rate(4);
            while (nh.ok()) {
                pub.publish (filteredCloud);
                ros::spinOnce ();
                loop_rate.sleep ();
            }

    	first_time = true;
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

/*
    //Publsher
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std::vector<std::vector<geometry_msgs::Pose> > >("poses", 1);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        pub.publish(pose_2Dcontainer);
        loop_rate.sleep();
    }
*/
    //ros::spin();
}


