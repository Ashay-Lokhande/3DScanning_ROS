
#ifndef VIEWSET_OBJECT_H_
#define VIEWSET_OBJECT_H_

#include "Pose_Object.h"

typedef std::vector<std::vector<bool>> booleanMatrix;

class ViewSet_Object {

      std::vector<Pose_Object> poses_in_set;   // Vector of the poses in the set
      double cost;			       // Cost of movement between poses (orientation change is negligable)
      booleanMatrix combinedMatrix;  	       // Combined boolMatrices
      double total_percent;		       // Total percent of object viewable from poses

      public:
      	viewSet_object(Pose_Object pobj);
};