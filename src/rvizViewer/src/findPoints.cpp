#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <math.h>
#include <geometry_msgs/Pose.h>
// #include <unordered_set>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

float calculate_slope(float pt_x, float pt_y, float pt_z, float pt2_x, float pt2_y, float pt2_z)
{
    float rise = pt2_z - pt_z;

    // printf("Point 1: %f, %f. %f\n", pt_x, pt_y, pt_z);
    // printf("Point 2: %f, %f, %f\n", pt2_x, pt2_y, pt2_z);

    float square = pow((pt2_x - pt_x), 2) + pow((pt2_y - pt_y), 2);
    float run = sqrt(square);
    // printf("What is the slope?: %f\n", rise/run);

    return rise/run;
}   

bool onLine(float view_x, float view_y, float view_z, float pt_x, float pt_y, float pt_z, float check_pt_x, float check_pt_y, float check_pt_z)
{
    // Calculate the slope from the view to the check_pt
    float slope_pt_to_view = calculate_slope(view_x, view_y, view_z, check_pt_x, check_pt_y, check_pt_z);

    // printf("Slope from view to check_pt: %f\n", slope_pt_to_view);

    // Calculate slope from the check_pt to the pt
    float slope_check_pt_to_pt = calculate_slope(view_x, view_y, view_z, pt_x, pt_y, pt_z);

    // printf("Slope from view to pt: %f\n", slope_check_pt_to_pt);
    // printf("Are they equivalent?: %d\n", temp);

    float delta = 0.0000000045f;

    float diff = fabs(slope_pt_to_view - slope_check_pt_to_pt);

    bool temp = (diff < delta);
    // printf("Difference: %f\n", diff);
    // printf("Boolean: %d\n", temp);

    //printf("Difference: %f\n", diff);
    // If they are equal return true
    return temp;
}


// method to find viewable points from a given view. 
// arg1: Passed in a pose that represents a point in space
// arg2: the point cloud object of the pcd file

// PEDNING: return the actual viewable points instead of just the percentage
// PENDING: consider orientation (various angles for a given point) when viewing the object
float findPoints(const geometry_msgs::Pose createdPoint, const PointCloud::ConstPtr& msg)
{


	// printing the current view point
    printf("Coordinates: %f, %f, %f\n", createdPoint.position.x, createdPoint.position.y, createdPoint.position.z);
    
    // build a hashset and put the slopes in here
    //std::unordered_set<float> seenSlopes;

    // x,y,z represents the cooridnates of the view 
    float x = createdPoint.position.x;
    float y = createdPoint.position.y;
    float z = createdPoint.position.z;

    // count of viewable points
    int count = 0;

    // total number of points in the point cloud
    int size = 0;

    // PENDING!!
    // idea is that for a given line from the view point, slope is the same
    // we want to count for a single point and exclude all the other points on that line
    // by that way we count for a single point and not count points behind the counted 
    // point

    // if slope is already contained, then check which point is closer
    // to the view point
    // BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points) {
    // 	float slope_pt_to_view = calculate_slope(x, y, z, pt.x, pt.y, pt.z);
    // 	seenSlopes.insert(slope_pt_to_view); // should eradicate all duplicates
    // 	size++; // counting total number of points viewed
    // }
    // return (100 * seenSlopes.size() / (float) (size)); // percentag of points viewed



    // old implementation!

    // traverse through points of the point cloud
    // for a given point we scan all the same points
    // for each pair of points, we check if they have the same slope or comparable slopes
    // if they are similar we discount this because we have seen this point before
    // by this way we count all the points, without duplicates
    // by doing numberInLine <=1 we make sure we count only unique points and not duplicates
    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points) {
        int numberInLine = 0;
        BOOST_FOREACH (const pcl::PointXYZ& pt2, msg->points) {
            bool onTheLine = onLine(x, y, z, pt.x, pt.y, pt.z, pt2.x, pt2.y, pt2.z);
            if (onTheLine) {
                numberInLine++;
            }
        }
        // printf("inView: %d\n", inView);
        if (numberInLine <= 1) {
            count++;
        }
        size++;
    }

    // printf("Count: %d\n", count);
    // printf("Size: %d\n", size);
    //printf("Percent of points viewed from pose: %f\n", 100 * count/(float)size);

    // finally return a ratio of the points seen to the total number of points 
    // in the pcl 
    return (100 * count / (float) (size));
}

// void callback(const PointCloud::ConstPtr& msg)
// {
//     // printf("Percent of points viewed: %f\n", findPoints(msg));
// }

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "generate_views");
//     ros::NodeHandle nh;
//     ros::Subscriber sub = nh.subscribe<PointCloud>("cloud", 1, callback);
//     ros::spin();
// }
