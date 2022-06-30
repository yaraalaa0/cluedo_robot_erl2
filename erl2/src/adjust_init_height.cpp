/**
 * \file position_service.cpp
 * \brief this file is an implementation of the random position server
 * \author Carmine Recchiuto
 * \version 0.1
 * \date 20/09/2021
 *
 * \details
 *
 * Services: <BR>
 *    /position_server
 *
 * Description :
 *
 * This node implements the random position server. When the server receives a request of 
 * minimum and maximum numbers, it replies with three random position values for x, y, and 
 * theta between the minimum and maximum limits. 
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
	 * \brief this is the callback function generates a random number within a range
	 * \param M defines the lower bownd of the range
	 * \param N defines the upper bound of the range
	 *
	 * \return random number of type double within the specified range
	 *
	 * This function takes as inout the min and max limits of the range and uses the rand() 
	 * function to generate a random number within the range 
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
	 * It initializes the node handle and the /position_server service, 
	 * and assigns myrandom() as a callback function to the service
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
