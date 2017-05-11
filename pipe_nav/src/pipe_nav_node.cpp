//==-- Includes
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


//==-- Global Variables
typedef enum {
	NAV_MODE_SEARCH = 0,
	NAV_MODE_ALIGN,
	NAV_MODE_DESCEND,
	NAV_MODE_SAMPLE,
	NAV_MODE_ERROR,
	NAV_MODE_NUM
} nav_mode_t;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped pose_current;
geometry_msgs::PoseStamped pose_out;

bool new_target_found = false;
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
	
    pose_out.pose.position.x = 0;
    pose_out.pose.position.y = 0;
    pose_out.pose.position.z = 0;

	//==-- Wait for FCU
    while(ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

	//==-- Main Loop
    while( ros::ok() ){
		switch( nav_mode ) {
			case NAV_MODE_SEARCH: {
				if( new_target_found ) {
					if( current_state.mode != "OFFBOARD" ) {
						if( set_mode_client.call(set_mode_offb) &&
							set_mode_offb.response.success) {
							
							ROS_INFO("Offboard enabled");
							nav_mode = NAV_MODE_ALIGN;
							new_target_found = false;
						}
					}
				}

				pose_out = pose_current;
				
				break;
			}
			case NAV_MODE_ALIGN: {
				ROS_INFO("Aligning Target");
				if (pose_out != pose_current){
					NAV_TRANSITION(pose_out, pose_current);
				}
				pose_out = pose_current;
				break;
			}
			case NAV_MODE_DESCEND: {
				ROS_INFO("Descending To Target");
				if (pose_out != pose_current){
					NAV_TRANSITION(pose_out, pose_current);
				}
				pose_out = pose_current;
				break;
			}
			case NAV_MODE_SAMPLE: {
				ROS_INFO("Sampling Target");
				HOVER_HOLD(pose_out, pose_current);
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
		
        local_pos_pub.publish(pose_out);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


void NAV_TRANSITION(

