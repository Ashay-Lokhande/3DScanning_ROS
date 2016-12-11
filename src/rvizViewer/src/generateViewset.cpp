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
 #include "ViewSet_Object.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef std::vector<std::vector<finalFilteredCloud> > view_info_array;
typedef std::map<int, point_viewed> boolean_struct ;

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

void callback(const PointCloud::ConstPtr& msg)
{
    geometry_msgs::Pose origin;

    // Calls generate views and expects all pose object info returned
    view_info_array views = generateViews(msg);
    ViewSet_Object viewSets[5];

    for(int index = 0; index < 5; index++)
    {
        ViewSet_Object ob = ViewSet_Object(views.at(0).at(index));
        viewSets[index] = ob;
        origin = views.at(0).at(index).viewedFrom;

        while(viewSets[index].total_percent <= 95.0)
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
            // for(std::vector<finalFilteredCloud> current_vector : views)
            // {

            //     for(finalFilteredCloud current_view : current_vector)
            //     {

            //     }
            // }

            viewSets[index].update(best_struct, best_percentage, best_combined);
            
        }
    }
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
