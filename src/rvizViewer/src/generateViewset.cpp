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
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/publisher.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/tf.h>
 #include "findPoints.h"
 #include "generateViews.h"
 #include "ViewSet_Object.h"

using namespace std;


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef std::vector<std::vector<finalFilteredCloud> > view_info_array;
typedef std::map<int, point_viewed> boolean_struct;

bool firstTime = true;

void callback(const PointCloud::ConstPtr& msg);

/*
    Pre-Conditions:
        boolean_struct x and y should be of the same size
    Post-Conditions: 
        Alters the boolean_struct object that is passed in to parameter x
*/
boolean_struct combine_booleans(boolean_struct x, boolean_struct y)
{
    std::map<int, point_viewed>::iterator it1;
    std::map<int, point_viewed>::iterator it2;

    it1 = x.begin();
    it2 = y.begin();

    while(it1 != x.end() && it2 != y.end())
    {
        if(it1->second.is_viewed != 1 && it2->second.is_viewed == 1)
        {
            it1->second.is_viewed = 1;
        }

        it1++;
        it2++;
    }

    return x;
}

double find_percent(boolean_struct z)
{
    int numViewed = 0;
    std::map<int, point_viewed>::iterator it;

    for(it = z.begin(); it != z.end(); it++)
    {
        if(it->second.is_viewed == 1)
            numViewed++;
    }

    return ((float) numViewed) / z.size() * 100;
}

// Distance is used to make sure that the pose we addd will always be the one clo
double findDist(geometry_msgs::Pose one, geometry_msgs::Pose two)
{
    double x,y,z;
    x = pow(one.position.x - two.position.x, 2);
    y = pow(one.position.y - two.position.y, 2);
    z = pow(one.position.z - two.position.z, 2);

    return sqrt(x + y + z);
}

 int main(int argc, char **argv)
 {
     //Subscriber 
     ros::init(argc, argv, "generate_views");
     ros::NodeHandle nh;
     ros::Subscriber sub = nh.subscribe<PointCloud>("cloud", 1, callback);
     ros::spin();

     //ros::spin();
 }

void callback(const PointCloud::ConstPtr& msg)
{
    geometry_msgs::Pose origin;
    // Calls generate views and expects all pose object info returned
    view_info_array views = generateViews(msg);
    ViewSet_Object viewSets[5];

    if (firstTime) {

        for(int index = 0; index < 5; index++)
        {
            
            viewSets[index] = ViewSet_Object(views.at(0).at(index));
            origin = views.at(0).at(index).viewedFrom;
            
            while(viewSets[index].total_percent <= 99.95) // required percentage to be viewed
            {
                finalFilteredCloud best_struct;
                boolean_struct best_combined;
                double best_percentage = 0;
                double distance = 0;


                view_info_array::iterator it;
                for(it = views.begin(); it != views.end(); it++){
                    std::vector<finalFilteredCloud>::iterator innerIt;
                    for(innerIt = (*it).begin(); innerIt != (*it).end(); innerIt++){

                        boolean_struct current_combined = combine_booleans(viewSets[index].combinedMatrix, (*innerIt).point_in_cloud);
                        double current_percentage = find_percent(current_combined); 
                        double cdist = findDist(origin, (*innerIt).viewedFrom);

                        if((current_percentage > best_percentage) || (current_percentage == best_percentage && distance > cdist))
                        {
                            best_percentage = current_percentage;
                            distance = cdist;
                            best_struct = (*innerIt);
                            best_combined = current_combined;
                        }

                    }
                }

                viewSets[index].dist_travelled = distance;
                viewSets[index].update(best_struct, best_percentage, best_combined);
                
            }
        }
    }

    firstTime = false;

    printf("Publishing starts!\n");

    ViewSet_Object result = viewSets[0]; // all elements in the array have same accurary, will change when angles is implemented

    for(int count = 0; count < 5; count++){
        printf("%f, number of views: %d, Distance traveled: %f\n", viewSets[count].total_percent, viewSets[count].view_info_set.size(), viewSets[count].dist_travelled);
    }

    ros::NodeHandle nh;
    string publishTopicFilteredCloud1 = "filteredCloudA";
    string publishTopicFilteredCloud2 = "filteredCloudB";
    string publicTopicViewablePoint1 = "viewedFromA";
    string publicTopicViewablePoint2 = "viewedFromB";
    ros::Publisher cloudPub1 = nh.advertise<PointCloud> (publishTopicFilteredCloud1, 1);
    ros::Publisher cloudPub2 = nh.advertise<PointCloud> (publishTopicFilteredCloud2, 1);
    
    // getting ready to advertise the view point
    

    geometry_msgs::PoseArray poseArray;
    poseArray.header.frame_id = "/map";
    ros::Publisher poseArrayPub = nh.advertise<geometry_msgs::PoseArray> (publicTopicViewablePoint1, 1);
    ros::Publisher viewPointPub1 = nh.advertise<geometry_msgs::PoseStamped> ("temp", 1);
    ros::Publisher viewPointPub2 = nh.advertise<geometry_msgs::PoseStamped> (publicTopicViewablePoint2, 1);

    // setting up PoseStamped object to publish
    geometry_msgs::PoseStamped publishedPoint1;
    geometry_msgs::PoseStamped publishedPoint2;
    publishedPoint1.header.frame_id = "/map";
    publishedPoint2.header.frame_id = "/map";
    publishedPoint1.pose = result.view_info_set.at(0).viewedFrom;
    publishedPoint2.pose = result.view_info_set.at(1).viewedFrom;

    for(int v = 0; v < result.view_info_set.size(); v++){
        poseArray.poses.push_back(result.view_info_set.at(v).viewedFrom);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud1 = result.view_info_set.at(0).cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud2 = result.view_info_set.at(1).cloud;

    ros::Rate loop_rate(4);
    while (nh.ok()) {
        poseArrayPub.publish(poseArray);
        loop_rate.sleep ();
    }
 }
