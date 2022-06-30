/**
 * \file go_to_waypoint.cpp
 * \brief this file is an implementation of the go to waypoint PDDL action
 * \author Yara Abdelmottaleb
 * \version 0.1
 * \date 30/06/2022
 *
 * \details
 *
 * Clients: <BR>
 *   reaching_goal
 *  
 *   
 *
 * Description :
 *
 * This node implements the interface for the PDDL action go to waypoint
 * It drives the robot to a given waypoint
 * 
 *
 *
*/

#include "erl2/go_to_waypoint.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <erl2/PlanningAction.h>


namespace KCL_rosplan {
        /**
	 * \brief this is the initialization function of GoToWaypointInterface class
	 * This function takes as input the node handle and initiliazes an instance of the class
	*/
	GoToWaypointInterface::GoToWaypointInterface(ros::NodeHandle &nh) {
			// here the initialization
	}
	
	/**
	 * \brief this is the callback function for the go to waypoint action interface
	 * 
	 *
	 * \return true when the action is complete
	 *
	 * This function gets the named goal waypoint and calls the action service with the corresponding goal position to drive the robot to this waypoint
	 * After reaching the goal, it returns true
	 * 
	*/
	bool GoToWaypointInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		// here the implementation of the action
		std::cout << "Going from " << msg->parameters[0].value << " to " << msg->parameters[1].value << std::endl;
		
		actionlib::SimpleActionClient<erl2::PlanningAction> ac("reaching_goal", true);
		erl2::PlanningGoal goal;
		ac.waitForServer();
		if(msg->parameters[1].value == "wp1"){
		goal.target_pose.pose.position.x = 2.4;
		goal.target_pose.pose.position.y = 0.0;
		goal.target_pose.pose.orientation.w = 0.0;
		}
		else if (msg->parameters[1].value == "wp2"){
		goal.target_pose.pose.position.x = 0.0;
		goal.target_pose.pose.position.y = 2.4;
		goal.target_pose.pose.orientation.w = 0.0;
		}
		else if (msg->parameters[1].value == "wp3"){
		goal.target_pose.pose.position.x = -2.4;
		goal.target_pose.pose.position.y = 0.0;
		goal.target_pose.pose.orientation.w = 0.0;
		}
		else if (msg->parameters[1].value == "wp4"){
		goal.target_pose.pose.position.x = 0.0;
		goal.target_pose.pose.position.y = -2.4;
		goal.target_pose.pose.orientation.w = 0.0;
		}
		else if (msg->parameters[1].value == "wp0"){
		goal.target_pose.pose.position.x = 0.0;
		goal.target_pose.pose.position.y = 0.0;
		goal.target_pose.pose.orientation.w = 0.0;
		}
		ac.sendGoal(goal);
		ac.waitForResult();
		
		ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
		return true;
	}
}


        /**
         * \brief this is main function of the node
         * It initializes the node handle, the action interface, and the service clients 
         *
        */
	int main(int argc, char **argv) {
		ros::init(argc, argv, "GoToWaypoint_rosplan_action", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");
		KCL_rosplan::GoToWaypointInterface my_aci(nh);
		my_aci.runActionInterface();
		return 0;
	}
