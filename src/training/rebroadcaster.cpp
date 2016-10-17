/*
 * rebroadcaster.cpp
 *
 *  Created on: Oct 17, 2016
 *      Author: gmartin
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <mrsmap/utilities/utilities.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <rosmrsmap/Broadcaster.h>

using namespace mrsmap;

class Rebroadcaster {
public:

	Rebroadcaster(ros::NodeHandle& nh) :
			nh_(nh),publish_(false) {

		publish_service_ = nh.advertiseService("start",
						&Rebroadcaster::broadcastRequest, this);

		stop_service_ = nh.advertiseService("stop",
								&Rebroadcaster::stopBroadcast, this);

	}

	bool broadcastRequest(rosmrsmap::Broadcaster::Request &req,
				rosmrsmap::Broadcaster::Response &res) {



		tf::poseMsgToTF(req.pose,tf_);
		tf_.frame_id_ = req.frame_id;
		tf_.child_frame_id_=req.child_frame_id;
		publish_=true;
		return true;

	}

	bool stopBroadcast(rosmrsmap::Broadcaster::Request &req,
					rosmrsmap::Broadcaster::Response &res) {
		ROS_INFO("stopping the rebroadcaster service..");
		publish_=false;
		return true;
	}





	void update(){

		if(!publish_)
			return;
		tf_.stamp_=ros::Time::now();
		tf_broadcaster_.sendTransform(tf_);

	}

	void stopTF(){
		publish_ = false;
	}

private:

	ros::NodeHandle nh_;
	tf::StampedTransform tf_;
	tf::TransformBroadcaster tf_broadcaster_;
	bool publish_;
	ros::ServiceServer publish_service_,stop_service_;
};


int main(int argc, char** argv) {

	ros::init(argc, argv, "rebroadcaster");
	ros::NodeHandle n("rebroadcaster");
	Rebroadcaster sm(n);

	ros::Rate r(10);
	while (ros::ok()) {

		sm.update();
		ros::spinOnce();
		r.sleep();

	}

	return 0;
}
