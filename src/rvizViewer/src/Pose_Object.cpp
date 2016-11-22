#include "pose_object.h"
#include <geometry_msgs/Pose.h>

typedef std::vector<std::vector<bool>> booleanMatrix;

Pose_Object::Pose_Object(Pose input, double percent, booleanMatrix bmat)
{
	current_Pose = input;
	percent_viewable = percent;
	boolMatrix = bmat;
}
