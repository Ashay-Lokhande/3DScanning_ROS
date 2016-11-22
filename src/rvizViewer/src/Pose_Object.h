#ifndef POSE_OBJECT_H_
#define POSE_OBJECT_H_

#include <geometry_msgs/Pose.h>

typedef std::vector<std::vector<bool>> booleanMatrix;

class Pose_Object {

      double percent_viewable;   // Length of a box
      booleanMatrix boolMatrix;  // Breadth of a box
      Pose current_Pose;   // Height of a box

      public:
  	  Pose_Object(Pose input, double percent, booleanMatrix bmat);
};