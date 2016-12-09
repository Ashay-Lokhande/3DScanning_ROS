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
#include <cmath>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

#define PI 3.141592653589793238462643383279502884197169
#define CAMERA_HEIGHT 0;

//count is a varuiable used to make sure that the node only generates the poses once
bool first_time = true;
//A two dimensional vector used to hold the poses generated around the object
std::vector<std::vector<geometry_msgs::Pose> > pose_2Dcontainer;

geometry_msgs::Point center;

void callback(const PointCloud::ConstPtr& msg)
{
    
    if(first_time)
    {
    	int i = 0, j = 0;
        float numPoints = 0;
    	printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);

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
    	//Finds the points of the circle around the center of the object	
    	
    	/*for (float theta = 0; theta < 2*PI; theta += 0.175){
    		float x = circle_radius * (float) cos(theta);
    		float y = circle_radius * (float) sin(theta);
    		std::vector<geometry_msgs::Pose> new_coordinate_pose;
    		//for loop for the 5 different orientations of the poses
    		for(float pitch_angle = 30; pitch_angle <= 150; pitch_angle += 30){
    			geometry_msgs::Pose createdPoint;
    			createdPoint.position.x = x;
    			createdPoint.position.y = y;
    			createdPoint.position.z = CAMERA_HEIGHT;
    			createdPoint.orientation.w = 0;
    			createdPoint.orientation.x = 0;
    			createdPoint.orientation.y = (float) sin(.5 * pitch_angle);
    			createdPoint.orientation.z = (float) sin(.5 * theta); //replace with degree change to look at the center

    			//Add the calculated pose to the array of poses
    			new_coordinate_pose.push_back(createdPoint);
                printf ("Percent of object viewed is: %f\n", findPoints(createdPoint, msg));
    		}
    		//Add the vector of poses containing the same position but different orientations
    		pose_2Dcontainer.push_back(new_coordinate_pose);
    	} 
		*/

    	for ( i -= circle_radius; i < circle_radius; i++)
        {
            for (j -= circle_radius; j < circle_radius; j++)
            {
                int condition_code = abs( i*i + j*j - circle_radius*circle_radius);
                if (condition_code < 5)
                {
                    std:std::vector<geometry_msgs::Pose> new_coordinate_pose;
                    for (float pitch_angle = 30; pitch_angle <= 150; pitch_angle += 30)
                    {
                        //angle to orient the pose to face the center of the circle
                        double theta = atan2(j - center.y, i -center.x);
                        //theta = theta * 180 / PI; //convert to degrees

                        geometry_msgs::Pose createdPoint;
                        createdPoint.position.x = i;
                        createdPoint.position.y = j;
                        createdPoint.position.z = CAMERA_HEIGHT;
                        createdPoint.orientation.w = 0;
                        createdPoint.orientation.x = 0;
                        createdPoint.orientation.y = (float) sin(.5 * pitch_angle);
                        createdPoint.orientation.z = (float) sin(.5 * theta); //replace the orientation z to face towards center

                        new_coordinate_pose.push_back(createdPoint);
                        printf("Percent of the object viewed is: %f\n", findPoints(createdPoint, msg) );
                    }
                    pose_2Dcontainer.push_back(new_coordinate_pose);
                    //printf("*");
                } /* else {
                        printf(" ");
                     } */
            }
            //printf("\n");
        }	            

    	first_time = false;
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


