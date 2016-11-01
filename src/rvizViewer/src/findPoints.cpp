#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <math.h>

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

    float delta = 0.0000045f;

    float diff = fabs(slope_pt_to_view - slope_check_pt_to_pt);

    bool temp = (diff < delta);
    // printf("Difference: %f\n", diff);
    // printf("Boolean: %d\n", temp);

    //printf("Difference: %f\n", diff);
    // If they are equal return true
    return temp;
}

void callback(const PointCloud::ConstPtr& msg)
{
    // printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    // Coordinates of the view pose relative to the center of the object
    float x = 5.0;
    float y = 5.0;
    float z = 5.0;
    int count = 0;
    int size = 0;
    // Compare for each point in the pcd
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

    printf("Percent of points viewed from pose: %f\n", 100 * count/(float)size);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "generate_views");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<PointCloud>("cloud", 1, callback);
    ros::spin();
}