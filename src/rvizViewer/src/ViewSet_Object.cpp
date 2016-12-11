#include "ViewSet_Object.h"


ViewSet_Object::ViewSet_Object()
{
  
}
ViewSet_Object::ViewSet_Object(finalFilteredCloud view)
{
	view_info_set.push_back(view);
	cost = 0;
	combinedMatrix = view.point_in_cloud;
	total_percent = view.percentageViewed;
  dist_travelled = 0;
}

void ViewSet_Object::update(finalFilteredCloud best_struct, double best_percentage, boolean_struct best_combined)
{
	view_info_set.push_back(best_struct);
	combinedMatrix = best_combined;
	total_percent = best_percentage;
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