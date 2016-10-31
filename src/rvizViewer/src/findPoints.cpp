/*
   This .cpp file will be used to find all the points in the published .pcd object 
   model that are viewable from the pose passed in as a parameter

Attributes:

- Has no main
- filled with functions that are mainly private. The only public method will
be a method called from generate views and returns our quantification value
that indicates the percent of points on the object that are seen by that view

To do:
- need to subscirbe to the topic we are pushing the .pcd pointcloud to in
the file rvizView

 */

#include <findPoints.h>


class findPoints(){

    //Public function to get the pose from generateViews.cpp
    public void quantifyDataInView(const geometry_msgs::Pose &current_view)_{

    	//subscibes to the cloud topic containing the pointcloud of the object we are scanning
    	// MAY HAVE TO CHANGE /cloud to cloud
    	ros::Subscriber sub = n.subscribe("/cloud", 1000, cloudCallback);

        //call functions
        	//type cone cone = makeCone(*current_view);
			/*
			Looping structure that generates lines from the pose and within the cone's width view
				loop:
					contains a total value to count the number of lines that hit a point on the object
					calls function and passes the line, function checks weather or not it hits the object we are scanning
			*/

        //retirn vaue whihc will be the percent (lines that hit / lines generated)
    }


    // in preivous method we subscribe to the same topic 
    // to which the pcd object has been published to
    // we subscribe to that topic "/cloud" and expect the callBackFunction to get passed
    // the pcd object
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pcdObject){

    	// we get the pcd object -- like the spray bottle, etc


  		
	}

    private [type of cone] makeCone(pose){

        //returns generated cone stucture at the pise
    }

    private boolean validLine(/*The line object, the .pcd object*/){

    	//checks to see if the line hits a point on the object 
    	//returns t/f depending on if it hits or not
    }


}
