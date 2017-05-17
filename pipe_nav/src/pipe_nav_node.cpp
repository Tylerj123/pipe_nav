//==-- Includes
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ml_msgs/MarkerDetection.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>

//==-- Global Variables
typedef enum {
	NAV_MODE_SEARCH = 0,
	NAV_MODE_ALIGN,
	NAV_MODE_DESCEND,
	NAV_MODE_SAMPLE,
	NAV_MODE_ERROR,
	NAV_MODE_NUM
} nav_mode_t;

tf2_ros::Buffer tfBuffer;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped pose_current;
geometry_msgs::PoseStamped marker_current;
geometry_msgs::Pose pose_goal;
geometry_msgs::PoseStamped pose_out;

std::vector< int > marker_ids_found;
//v.push_back(item);
//v.at(index);
//v.size()
int marker_id_current = -1;


//==-- Callbacks
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    pose_current = *msg;
}

void marker_cb(const ml_msgs::MarkerDetection::ConstPtr& msg) {
    for (int i = 0; i < msg->markers.size(); i++) {
		if (marker_id_current == -1) {
			//ROS_INFO("Looking for new marker");
			bool new_marker = true;
			for (int j = 0; j < marker_ids_found.size(); j++) {
				if(marker_ids_found.at(j) == msg->markers.at(i).marker_id) {
					new_marker = false;
				}
			}

			if (new_marker) {
				marker_id_current = msg->markers.at(i).marker_id;
				ROS_INFO("Set new target (id: %i)", marker_id_current);
			}
		}

		if (marker_id_current == msg->markers.at(i).marker_id) {
			//ROS_INFO("Updating target");
			marker_current.header = msg->header;
			marker_current.pose = msg->markers.at(i).pose;
		}
	}

}

// Determines if the position of the UAV has reached the goal position
bool waypoint_reached() {

	float x = pose_goal.position.x - pose_current.pose.position.x;
	float y = pose_goal.position.y - pose_current.pose.position.y;
	float z = pose_goal.position.z - pose_current.pose.position.x;

	return (sqrt(x*x + y*y + z*z) < 0.2);
}

//==-- Main
int main(int argc, char **argv) {
	//==-- Node Setup
    ros::init(argc, argv, "pipe_nav");
    ros::NodeHandle nh;

	//==-- Publishers & Subscribers
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, pose_cb);

	ros::Subscriber marker_sub = nh.subscribe<ml_msgs::MarkerDetection>
            ("/ml_landmarks/detected_markers", 10, marker_cb);

	//==-- Services
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");

	//==-- Variables
    ros::Rate rate(20.0);	//The setpoint publishing rate MUST be faster than 2Hz

	nav_mode_t nav_mode = NAV_MODE_SEARCH;

	mavros_msgs::SetMode set_mode_offb;
    set_mode_offb.request.custom_mode = "OFFBOARD";
    mavros_msgs::SetMode set_mode_mission;
    set_mode_mission.request.custom_mode = "MISSION";
    mavros_msgs::SetMode set_mode_failsafe;
    set_mode_failsafe.request.custom_mode = "FAILSAFE";

	tf2_ros::TransformListener tfListener(tfBuffer);
	tf2_ros::TransformBroadcaster tfbr;

    pose_goal.position.x = 0;
    pose_goal.position.y = 0;
    pose_goal.position.z = 0;
    pose_goal.orientation.w = 1.0;
    pose_goal.orientation.x = 0;
    pose_goal.orientation.y = 0;
    pose_goal.orientation.z = 0;

	bool sample_inprogress = false;
	ros::Time sample_begin_time;
	pose_out.header.frame_id = "world";
	ROS_WARN("Setup Completed Waiting for FCU");
	//==-- Wait for FCU
    while(ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }


	ROS_INFO("FCU Connected - Searching for Target");
	//==-- Main Loop
    while( ros::ok() ){
		if( marker_id_current != -1 ) {
			try{

				//ROS_INFO("Estimating Target Position");

				geometry_msgs::TransformStamped tmp_marker;
				tmp_marker.header.stamp = ros::Time::now();
				tmp_marker.header.frame_id = "camera";
				tmp_marker.child_frame_id = "target_marker";
				tmp_marker.transform.translation.x = marker_current.pose.position.x;
				tmp_marker.transform.translation.y = marker_current.pose.position.y;
				tmp_marker.transform.translation.z = marker_current.pose.position.z;
				tmp_marker.transform.rotation.w = marker_current.pose.orientation.w;
				tmp_marker.transform.rotation.x = marker_current.pose.orientation.x;
				tmp_marker.transform.rotation.y = marker_current.pose.orientation.y;
				tmp_marker.transform.rotation.z = marker_current.pose.orientation.z;
				tfBuffer.setTransform(tmp_marker, "nav_node_temp");

				geometry_msgs::TransformStamped marker_world = tfBuffer.lookupTransform("target_marker", "world",tmp_marker.header.stamp);

				//Broadcast Maker Tranform
				tfbr.sendTransform(tmp_marker);

				pose_goal.position.x = marker_world.transform.translation.x;
				pose_goal.position.y = marker_world.transform.translation.y;
				pose_goal.position.z = marker_world.transform.translation.z + 1; //XXX Hover Height added
				pose_goal.orientation.w = marker_world.transform.rotation.w;
				pose_goal.orientation.x = marker_world.transform.rotation.x;
				pose_goal.orientation.y = marker_world.transform.rotation.y;
				pose_goal.orientation.z = marker_world.transform.rotation.z;

			}
			catch (tf2::TransformException &ex) {
			  ROS_WARN("%s",ex.what());
				continue;
			}
		}

		switch( nav_mode ) {
			case NAV_MODE_SEARCH: {
				if( marker_id_current != -1 ) {
					ROS_INFO_THROTTLE(2.0, "Pausing search, taking control");
					if( current_state.mode == "MISSION" ) {
						if( set_mode_client.call(set_mode_offb) &&
							set_mode_offb.response.success) {

							ROS_INFO("Offboard enabled");
							nav_mode = NAV_MODE_ALIGN;
						}
					}
				}

				pose_goal = pose_current.pose;

				break;
			}
			case NAV_MODE_ALIGN: {
				ROS_INFO("Aligning Target");

				if (waypoint_reached()) {
					nav_mode = NAV_MODE_DESCEND;
				}

				break;
			}
			case NAV_MODE_DESCEND: {
				ROS_INFO("Descending To Target");
				if (waypoint_reached()) {
					nav_mode = NAV_MODE_SAMPLE;
				}

				break;
			}
			case NAV_MODE_SAMPLE: {
				ROS_INFO("Sampling Target");
				if (!sample_inprogress) {
					sample_begin_time = ros::Time::now();
					sample_inprogress = true;

				}

				if (sample_inprogress && ((ros::Time::now() - sample_begin_time).toSec() > 10.0)) {
					// set mode mission
					if( set_mode_client.call(set_mode_mission) &&
						set_mode_mission.response.success) {

						ROS_INFO("Return to Mission Mode");
						nav_mode = NAV_MODE_SEARCH;
					} else {
						ROS_ERROR("Failed to return to mission");
						nav_mode = NAV_MODE_ERROR;
					}

					// Marking targets sampled
					marker_ids_found.push_back(marker_id_current);
					marker_id_current = -1;
					sample_inprogress = false;
				}

				break;
			}
			default: {
				ROS_ERROR("ERROR DETECTED, ATTEMPTING FAILSAFE");

				if( set_mode_client.call(set_mode_failsafe) &&
					set_mode_failsafe.response.success) {
					ROS_INFO("FAILSAFE ENABLED");
				} else {
					ROS_ERROR("COULD NOT ENTER FAILSAFE MODE");
				}

				ros::shutdown();

				break;
			}
		}

		pose_out.header.stamp = ros::Time::now();
		pose_out.pose = pose_goal;

        local_pos_pub.publish(pose_out);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


