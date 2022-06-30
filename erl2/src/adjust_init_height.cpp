/**
 * \file adjust_init_height.cpp
 * \brief this file is an implementation of the adjust initial height action
 * \author Yara Abdelmottaleb
 * \version 0.1
 * \date 30/06/2022
 *
 * \details
 *
 * Description :
 *
 * This node implements the interface for the action adujst initial height
 * It sets the robot arm to a given height using moveit
 * 
 *
 *
*/

#include "erl2/adjust_init_height.h"
#include <unistd.h>


namespace KCL_rosplan {
        
	/**
	 * \brief this is the initialization function of AdjustInitHeightInterface class
	 * This function takes as input the node handle and initiliazes an instance of the class
	*/
	AdjustInitHeightInterface::AdjustInitHeightInterface(ros::NodeHandle &nh) {
			// here the initialization
	}
        /**
	 * \brief this is the callback function for the ajust initial height action interface
	 * \param msg->parameters[0] defines the desired height to put the arm at (either h1 or h2)
	 * 
	 *
	 * \return true when the action is complete
	 *
	 * This function takes as input the desired height for the robot arm and calls moveit to set the robot arm at this height
	 * 
	*/
	bool AdjustInitHeightInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		// here the implementation of the action
		std::cout << "Adjusting Initial Arm Height to " << msg->parameters[0].value << std::endl;
		
		robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
		const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
		ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
		
		moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
		kinematic_state->setToDefaultValues();
		const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
		moveit::planning_interface::MoveGroupInterface group("arm");
		const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
		
		group.setStartStateToCurrentState();
		if (msg->parameters[0].value == "h1"){
		    group.setNamedTarget("low");
                    group.move();
		    }
		if (msg->parameters[0].value == "h2"){
		    group.setNamedTarget("high");
                    group.move();
		    }
		
	       sleep(5);
	       ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
	       return true;
	}
}

        /**
	 * \brief this is main function of the node
	 *
	 * It initializes the node handle and the action interface, 
	 *
	*/
	int main(int argc, char **argv) {
		ros::init(argc, argv, "AdjustInitHeight_rosplan_action", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");
		
		ros::AsyncSpinner spinner(1);
		spinner.start();
		
  
		KCL_rosplan::AdjustInitHeightInterface my_aci(nh);
		my_aci.runActionInterface();
		return 0;
	}
