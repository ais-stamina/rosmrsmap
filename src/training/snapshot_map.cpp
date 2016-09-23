/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Bonn, Computer Science Institute VI
 *  Author: Joerg Stueckler, 13.06.2013
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of University of Bonn, Computer Science Institute
 *     VI nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */


#include <ros/ros.h>
#include <ros/package.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/publisher.h>
#include <pcl_ros/transforms.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/query.h>
#include <rosbag/message_instance.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
//#include <rosmrsmap/ObjectData.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
//#include <rosmrsmap/SelectBoundingBox.h>
//#include <rosmrsmap/TriggerInitialAlignment.h>


#include <Eigen/Core>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>

#include <mrsmap/map/multiresolution_surfel_map.h>
#include <mrsmap/registration/multiresolution_surfel_registration.h>


#include <boost/thread/thread.hpp>
#include "pcl/common/common_headers.h"
#include "pcl/visualization/pcl_visualizer.h"


#include "pcl/common/centroid.h"
#include "pcl/common/eigen.h"


#include <rosmrsmap/StringService.h>
#include <rosmrsmap/ObjectPoseService.h>
#include <std_msgs/Int32.h>

#include <mutex>

using namespace mrsmap;



class SnapshotMap
{
public:

    SnapshotMap( ros::NodeHandle& nh ) : nh_( nh ),object_available(false) {

		imageAllocator_ = boost::shared_ptr< MultiResolutionSurfelMap::ImagePreAllocator >( new MultiResolutionSurfelMap::ImagePreAllocator() );
		treeNodeAllocator_ = boost::shared_ptr< spatialaggregate::OcTreeNodeDynamicAllocator< float, MultiResolutionSurfelMap::NodeValue > >( new spatialaggregate::OcTreeNodeDynamicAllocator< float, MultiResolutionSurfelMap::NodeValue >( 10000 ) );

		snapshot_service_ = nh.advertiseService("snapshot", &SnapshotMap::snapshotRequest, this);
		object_pose_service_ = nh.advertiseService("object_pose", &SnapshotMap::poseRequest, this);
		pub_cloud = pcl_ros::Publisher<pcl::PointXYZRGB>(nh, "output_cloud", 1);

		pub_status_ = nh.advertise< std_msgs::Int32 >( "status", 1 );

		tf_listener_ = boost::shared_ptr< tf::TransformListener >( new tf::TransformListener() );

		//added to debug
		m_debugObjPub = nh.advertise<geometry_msgs::PoseStamped>(
					"debug_object", 1, true);

		nh.param<double>( "max_resolution", max_resolution_, 0.0125 );
		nh.param<double>( "max_radius", max_radius_, 30.0 );

		nh.param<double>( "dist_dep_factor", dist_dep_, 0.005 );

		nh.param<std::string>( "map_folder", map_folder_, "." );

		nh.param<std::string>( "init_frame", init_frame_, "" );

		create_map_ = false;
		do_publish_tf_= true;

		responseId_ = -1;

    }

    bool poseRequest(rosmrsmap::ObjectPoseService::Request &req, rosmrsmap::ObjectPoseService::Response &res){

    	while(!object_available)
    		ros::Duration(1,0).sleep();
    	object_mutex.lock();
    	ROS_INFO("poseRequest got the mutex");
    	ROS_INFO_STREAM("poseRequest object_tf_="<<object_tf_.getOrigin().getX()<<" "<<object_tf_.getOrigin().getY()<<" "<<object_tf_.getOrigin().getZ());
    	res.responseId= responseId_ ;
    	res.object_name = object_name_;
    	res.pose_frame_id = init_frame_;
    	//nice way to transform from tf::Vector -> geometryMsgs/Pose
    	res.object_pose.position.x = object_tf_.getOrigin().getX();
    	res.object_pose.position.y = object_tf_.getOrigin().getY();
    	res.object_pose.position.z = object_tf_.getOrigin().getZ();

    	res.object_pose.orientation.x = object_tf_.getRotation().getX();
    	res.object_pose.orientation.y = object_tf_.getRotation().getY();
    	res.object_pose.orientation.z = object_tf_.getRotation().getZ();
    	res.object_pose.orientation.w = object_tf_.getRotation().getW();
    	object_mutex.unlock();
    	ROS_INFO("poseRequest released the mutex");
    	return true;
    }


	bool snapshotRequest( rosmrsmap::StringService::Request &req, rosmrsmap::StringService::Response &res ) {

		object_name_ = req.str;
		sub_cloud_ = nh_.subscribe( "input_cloud", 1, &SnapshotMap::dataCallback, this );
		create_map_ = true;

		ROS_INFO_STREAM("got request for object="<<object_name_<< " | subscribed at " << sub_cloud_.getTopic() );

		res.responseId = responseId_ + 1;

		return true;

	}

	bool normalise_transform(Eigen::Matrix3d& eigenvectors, const sensor_msgs::PointCloud2ConstPtr& point_cloud){


		geometry_msgs::PointStamped pt_0_base_link, pt_x_base_link;
		geometry_msgs::PointStamped pt_0_camera, pt_x_camera;

		//obtain base_link's (x) axis in camera coordinates
		pt_0_base_link.header.frame_id = init_frame_;
		pt_0_base_link.point.x = pt_0_base_link.point.y = pt_0_base_link.point.z = 0.;
		pt_x_base_link.header.frame_id = init_frame_;
		pt_x_base_link.point.x = 1.0;
		pt_x_base_link.point.y = pt_x_base_link.point.z = 0.;

		tf_listener_->transformPoint(point_cloud->header.frame_id,pt_0_base_link,pt_0_camera);
		tf_listener_->transformPoint(point_cloud->header.frame_id,pt_x_base_link,pt_x_camera);

		ROS_INFO_STREAM("pt_x_camera="<<pt_x_camera);
		ROS_INFO_STREAM("pt_0_camera="<<pt_0_camera);
		ROS_INFO_STREAM("point_cloud->header.frame_id="<<point_cloud->header.frame_id);

		//base_link_x is the (x) axis of the base link frame of ref. in camera coordinates
		Eigen::Vector3d base_link_x(pt_x_camera.point.x - pt_0_camera.point.x,pt_x_camera.point.y - pt_0_camera.point.y, pt_x_camera.point.z - pt_0_camera.point.z );

		//we now have to look for the axis in the eigenvectors that is most orthogonal to base_link_x
		double dot_x = fabs(Eigen::Vector3d(eigenvectors.col(2)).dot( base_link_x ));
		double dot_y = fabs(Eigen::Vector3d(eigenvectors.col(1)).dot( base_link_x ));

		if(dot_x > dot_y){
			ROS_INFO_STREAM("base_link_x="<<base_link_x);
			ROS_INFO_STREAM("object x="<<eigenvectors.col(2));
			ROS_INFO_STREAM("object y="<<eigenvectors.col(1));
			ROS_INFO_STREAM("object z="<<eigenvectors.col(0));

			// x eigenvector
			if( Eigen::Vector3d(eigenvectors.col(2)).dot( base_link_x ) > 0.0 ){
				ROS_INFO("the object's (x) axis points in the same direction as base_link's (x) axis");
				return false;
			}
			else{
				ROS_INFO("the object's (x) axis points in opposite direction as base_link's (x) axis");
				return true;
			}
		}
		else {

			ROS_INFO_STREAM("base_link_x="<<base_link_x);
			ROS_INFO_STREAM("object x="<<eigenvectors.col(2));
			ROS_INFO_STREAM("object y="<<eigenvectors.col(1));
			ROS_INFO_STREAM("object z="<<eigenvectors.col(0));

			// x eigenvector
			if( Eigen::Vector3d(eigenvectors.col(1)).dot( base_link_x ) > 0.0 ){
				ROS_INFO("the object's (y) axis points in the same direction as base_link's (x) axis");
				return false;
			}
			else{
				ROS_INFO("the object's (y) axis points in opposite direction as base_link's (x) axis");
				return true;
			}

		}



	}


	/*
	 * here comes the object cluster point cloud in camera frame of reference
	 */
	void dataCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud) {

		if( !create_map_ )
			return;

		ROS_INFO("creating map");
		object_mutex.lock();
		ROS_INFO("dataCallback got the mutex");


		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudIn = pcl::PointCloud<pcl::PointXYZRGB>::Ptr( new pcl::PointCloud<pcl::PointXYZRGB>() );
		pcl::fromROSMsg(*point_cloud, *pointCloudIn);

//		Eigen::Matrix4d objectTransform = Eigen::Matrix4d::Identity();

		// change reference frame of point cloud to point mean and oriented along principal axes
		Eigen::Vector4d mean;
		Eigen::Vector3d eigenvalues;
		Eigen::Matrix3d cov;
		Eigen::Matrix3d eigenvectors;
		pcl::computeMeanAndCovarianceMatrix( *pointCloudIn, cov, mean );
		pcl::eigen33( cov, eigenvectors, eigenvalues );

		/*
		 * Added comment to this misterious sign change:
		 * Assuming the eigenvectors of the object come in the camera frame of reference
		 * this means the unit vector along the Z axis (Eigen::Vector3d::UnitZ()) points away from the camera.
		 * The LAST eigenvector points upwards/downwards. If their dot product is positive
		 * means the eigenvector points downwards. This sign change would
		 * normalize it so that it always points downwards
		 *
		 */
		// z eigenvector
		if( Eigen::Vector3d(eigenvectors.col(0)).dot( Eigen::Vector3d::UnitZ() ) > 0.0 )
			eigenvectors.col(0) = (-eigenvectors.col(0)).eval();


		Eigen::Matrix4d objectTransform = Eigen::Matrix4d::Identity();
		objectTransform.block<3,1>(0,0) = eigenvectors.col(2);
		objectTransform.block<3,1>(0,1) = eigenvectors.col(1);
		objectTransform.block<3,1>(0,2) = eigenvectors.col(0);
		objectTransform.block<3,1>(0,3) = mean.block<3,1>(0,0);

		if( objectTransform.block<3,3>(0,0).determinant() < 0 ) {
			// x eigenvector
			objectTransform.block<3,1>(0,0) = -objectTransform.block<3,1>(0,0);
		}

		eigenvectors.col(2) = objectTransform.block<3,1>(0,0);
		eigenvectors.col(1) = objectTransform.block<3,1>(0,1);
		eigenvectors.col(0) = objectTransform.block<3,1>(0,2);
		if(normalise_transform(eigenvectors,point_cloud)){
			// x eigenvector
			objectTransform.block<3,1>(0,0) = -objectTransform.block<3,1>(0,0);
			// y eigenvector
			objectTransform.block<3,1>(0,1) = -objectTransform.block<3,1>(0,1);
		}

		// transform from camera frame of reference to object's main axes frame of reference
		Eigen::Matrix4d objectTransformInv = objectTransform.inverse();

		//the MRSmap of the object is stored in the frame of reference of the objects' eigen axis
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectPointCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr( new pcl::PointCloud<pcl::PointXYZRGB>() );
		pcl::transformPointCloud( *pointCloudIn, *objectPointCloud, (objectTransformInv).cast<float>() );

		objectPointCloud->sensor_origin_ = objectTransformInv.block<4,1>(0,3).cast<float>();
		objectPointCloud->sensor_orientation_ = Eigen::Quaternionf( objectTransformInv.block<3,3>(0,0).cast<float>() );
		
		treeNodeAllocator_->reset();
		map_ = boost::shared_ptr< MultiResolutionSurfelMap >( new MultiResolutionSurfelMap( max_resolution_, max_radius_, treeNodeAllocator_ ) );

		map_->params_.dist_dependency = dist_dep_;

		std::vector< int > pointIndices( objectPointCloud->points.size() );
		for( unsigned int i = 0; i < pointIndices.size(); i++ ) pointIndices[i] = i;
		map_->imageAllocator_ = imageAllocator_;
		map_->addPoints( *objectPointCloud, pointIndices );
		map_->octree_->root_->establishNeighbors();
		map_->evaluateSurfels();
		map_->buildShapeTextureFeatures();

		map_->save( map_folder_ + "/" + object_name_ + ".map" );

		if( init_frame_ != "" ) {

			ROS_INFO_STREAM( "Looking up transform to <init_frame>=" << init_frame_ <<" from <point_cloud->header.frame_id>="<<point_cloud->header.frame_id );

			try {

				tf::StampedTransform tf;
				tf_listener_->lookupTransform( init_frame_, point_cloud->header.frame_id, point_cloud->header.stamp, tf );
				Eigen::Affine3d init_frame_transform;
				tf::transformTFToEigen( tf, init_frame_transform );
				//obtains the transform of the object frame to the base_link frame of reference
				objectTransform = (init_frame_transform.matrix() * objectTransform).eval();


			}
				catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
			}


		}

		{

			Eigen::Quaterniond q( objectTransform.block<3,3>(0,0) );

			std::ofstream initPoseFile( map_folder_ + "/" + object_name_ + ".pose" );
			initPoseFile << "# x y z qx qy qz qw" << std::endl;
			initPoseFile << objectTransform(0,3) << " " << objectTransform(1,3) << " " << objectTransform(2,3) << " "
					 << q.x() << " " << q.y() << " " << q.z() << " "  << q.w() << std::endl;
			initPoseFile << "# init_pose: { position: { x: " << objectTransform(0,3) << ", y: " << objectTransform(1,3) << ", z: " << objectTransform(2,3)
					<< " }, orientation: { x: " << q.x() << ", y: " << q.y() << ", z: " << q.z() << ", w: " << q.w() << " } } }" << std::endl;
		}

		cloudv = pcl::PointCloud< pcl::PointXYZRGB >::Ptr( new pcl::PointCloud< pcl::PointXYZRGB >() );
		cloudv->header.frame_id = object_name_;
		map_->visualize3DColorDistribution( cloudv, -1, -1, false );


		Eigen::Quaterniond q( objectTransform.block<3,3>(0,0) );

		object_tf_.setIdentity();
		object_tf_.setRotation( tf::Quaternion( q.x(), q.y(), q.z(), q.w() ) );
		object_tf_.setOrigin( tf::Vector3( objectTransform(0,3), objectTransform(1,3), objectTransform(2,3) ) );

		//the object tf is from the camera frame of reference to the object's
		object_tf_.stamp_ = point_cloud->header.stamp;
		object_tf_.child_frame_id_ = object_name_;

		if( init_frame_ == "" ) {
			object_tf_.frame_id_ = point_cloud->header.frame_id;
		}
		else
			object_tf_.frame_id_ = init_frame_;

		first_publication_time = ros::Time::now();
		if(do_publish_tf_){
			//ROS_INFO_STREAM("object_tf_.frame_id_="<<object_tf_.frame_id_);
			//ROS_INFO_STREAM("> Snapshot map :dataCallback() publishing transform object_tf_.frame_id_ (init_frame_)=" << object_tf_.frame_id_ << " child="<< object_tf_.child_frame_id_<<std::endl);
//			std::cout <<"Press enter to continue..."<<std::endl;
//			std::cin.get();
			tf_broadcaster.sendTransform( object_tf_ );

		}

		sub_cloud_.shutdown();

		create_map_ = false;

		responseId_++;
		object_available = true;
		object_mutex.unlock();

		ROS_INFO("dataCallback released the mutex");

	}

	void update() {

		std_msgs::Int32 status;
		status.data = responseId_;
		pub_status_.publish( status );

		if( cloudv ) {

			object_tf_.stamp_ = ros::Time::now();
			if(do_publish_tf_){
				//ROS_INFO_STREAM("> Snapshot_map :update() object_tf_.frame_id_ (init_frame_)=" << object_tf_.frame_id_ << " child="<< object_tf_.child_frame_id_<<std::endl);
				//ROS_INFO_STREAM("> Snapshot map :update() publishing transform object_tf_.frame_id_ (init_frame_)=" << object_tf_.frame_id_ << " child="<< object_tf_.child_frame_id_<<std::endl);
//				std::cout <<"Press enter to continue..."<<std::endl;
//				std::cin.get();
				tf_broadcaster.sendTransform( object_tf_ );



				ros::Duration elapsed_time = ros::Time::now() - first_publication_time;
				if(elapsed_time.toSec()>100)
					do_publish_tf_ = false;
			}

			std_msgs::Header header;
			header.frame_id = object_name_;
			header.stamp = object_tf_.stamp_;
			cloudv->header = pcl_conversions::toPCL( header );
			pub_cloud.publish( cloudv );
		}


	}


public:

	ros::NodeHandle nh_;
	ros::Subscriber sub_cloud_;
	ros::Publisher pub_status_;
	pcl_ros::Publisher<pcl::PointXYZRGB> pub_cloud;
	boost::shared_ptr< tf::TransformListener > tf_listener_;
	tf::TransformBroadcaster tf_broadcaster;

	ros::Publisher m_debugObjPub;

	double max_resolution_, max_radius_, dist_dep_;

	bool create_map_, do_publish_tf_;
	ros::Time first_publication_time;

	ros::ServiceServer snapshot_service_,object_pose_service_;

	std::string map_folder_;
	std::string object_name_;
	std::string init_frame_;

	boost::shared_ptr< MultiResolutionSurfelMap > map_;
	tf::StampedTransform object_tf_;

	pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloudv;

	boost::shared_ptr< MultiResolutionSurfelMap::ImagePreAllocator > imageAllocator_;
	boost::shared_ptr< spatialaggregate::OcTreeNodeDynamicAllocator< float, MultiResolutionSurfelMap::NodeValue > > treeNodeAllocator_;

	int responseId_;
private:
	std::mutex object_mutex;
	bool object_available;

};



int main(int argc, char** argv) {

	ros::init(argc, argv, "snapshot_map");
	ros::NodeHandle n("snapshot_map");
	SnapshotMap sm( n );

	ros::Rate r( 100 );
	while( ros::ok() ) {

		sm.update();

		ros::spinOnce();
		r.sleep();

	}


	return 0;
}

