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