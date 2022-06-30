/**
 * \file move_arm.cpp
 * \brief this file is an implementation of the move arm PDDL action
 * \author Yara Abdelmottaleb
 * \version 0.1
 * \date 30/06/2022
 *
 * \details
 *
 *
 * Description :
 *
 * This node implements the interface for the PDDL action move arm
 * It 
 * 
 *
 *
*/

#include "erl2/move_arm.h"
#include <unistd.h>



namespace KCL_rosplan {
        /**
	 * \brief this is the initialization function of MoveArmInterface class
	 * This function takes as input the node handle and initiliazes an instance of the class
	*/
	MoveArmInterface::MoveArmInterface(ros::NodeHandle &nh) {
			// here the initialization
	}
	
	/**
	 * \brief this is the callback function for the move arm action interface
	 * 
	 * \param msg->parameters[0] defines the current height of the arm (either h1 or h2)
	 * \param msg->parameters[1] defines the desired height to move the arm to (either h1 or h2)
	 *
	 * \return true when the action is complete
	 *
	 * This function checks the given desired height and moves the robot arm to this height using moveit
	 * 
	*/
	bool MoveArmInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		// here the implementation of the action
		std::cout << "Moving Arm from " << msg->parameters[0].value << " to " << msg->parameters[1].value << std::endl;
		
		robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
		const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
		ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
		
		moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
		kinematic_state->setToDefaultValues();
		const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
		moveit::planning_interface::MoveGroupInterface group("arm");
		const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
		
		group.setStartStateToCurrentState();
		if (msg->parameters[1].value == "h1"){
		    group.setNamedTarget("low");
                    group.move();
		    }
		if (msg->parameters[1].value == "h2"){
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
		ros::init(argc, argv, "MoveArm_rosplan_action", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");
		
		ros::AsyncSpinner spinner(1);
		spinner.start();
		
  
		KCL_rosplan::MoveArmInterface my_aci(nh);
		my_aci.runActionInterface();
		return 0;
	}
