#ifndef VIEWSET_OBJECT_H
#define VIEWSET_OBJECT_H

// ttps://mirror.umd.edu/roswiki/doc/diamondback/api/geometry_msgs/html/msg/PoseArray.html
// #include <geometry_msgs/PoseArray.h>
#include "findPoints.h"

typedef std::map<int, point_viewed> boolean_struct;

class ViewSet_Object {

  public:
      std::vector<finalFilteredCloud> view_info_set;   // Vector of the views in the set
      double cost;			       // Cost of movement between poses (orientation change is negligable)
      boolean_struct combinedMatrix;  	       // Combined boolMatrices
      double total_percent;		       // Total percent of object viewable from poses
      double dist_travelled;         // Distance traveled for the views in the viewset
      //geometry_msgs::Pose[] pose_views;

      // Functions
      ViewSet_Object();
      ViewSet_Object(finalFilteredCloud view);
      void update(finalFilteredCloud best_struct, double best_percentage, boolean_struct best_combined);
};

#endif