#include "ViewSet_Object.h"

ViewSet_Object::ViewSet_Object(Pose_Object pobj)
{
	poses_in_set.push_back(pobj);
	cost = 0;
	combinedMatrix = pobj.boolMatrix;
}