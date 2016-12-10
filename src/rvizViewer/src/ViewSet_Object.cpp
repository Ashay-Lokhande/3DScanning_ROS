#include "ViewSet_Object.h"

ViewSet_Object::ViewSet_Object(finalFilteredCloud view)
{
	view_info_set.push_back(view);
	cost = 0;
	boolean_struct = view.point_in_cloud;
	total_percent = view.percentageViewed;
}



/*
std::map<int, point_viewed> boolean_struct;

class ViewSet_Object {

      std::vector<finalFilteredCloud> view_info_set;   // Vector of the views in the set
      double cost;			       // Cost of movement between poses (orientation change is negligable)
      boolean_struct combinedMatrix;  	       // Combined boolMatrices
      double total_percent;		       // Total percent of object viewable from poses
      //geometry_msgs::Pose[] pose_views;

      public:
      	viewSet_object(finalFilteredCloud view);
};



struct finalFilteredCloud {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    geometry_msgs::Pose viewedFrom;
    double percentageViewed;
    std::map<int, point_viewed> point_in_cloud;
};

*/