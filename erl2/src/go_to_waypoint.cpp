#include "erl2/go_to_waypoint.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <erl2/PlanningAction.h>


namespace KCL_rosplan {

	GoToWaypointInterface::GoToWaypointInterface(ros::NodeHandle &nh) {
			// here the initialization
	}

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

	int main(int argc, char **argv) {
		ros::init(argc, argv, "GoToWaypoint_rosplan_action", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");
		KCL_rosplan::GoToWaypointInterface my_aci(nh);
		my_aci.runActionInterface();
		return 0;
	}
