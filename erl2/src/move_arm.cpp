#include "erl2/move_arm.h"
#include <unistd.h>



namespace KCL_rosplan {

	MoveArmInterface::MoveArmInterface(ros::NodeHandle &nh) {
			// here the initialization
	}

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
		
                
		/*
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		geometry_msgs::Pose pose1;
		
		if (msg->parameters[2].value == "wp1"){
		    pose1.orientation.w = 0.70;
		    pose1.orientation.x = 0.00;
		    pose1.orientation.y = 0.00;
		    pose1.orientation.z = -0.70;
		    
		    pose1.position.x =  3.0;
		    pose1.position.y =  0.00;
		    }
		else if (msg->parameters[2].value == "wp2"){
		    pose1.orientation.w = 0.70;
		    pose1.orientation.x = 0.00;
		    pose1.orientation.y = 0.00;
		    pose1.orientation.z = 0.00;
		    
		    pose1.position.x =  0.0;
		    pose1.position.y =  3.0;
		    }
		else if (msg->parameters[2].value == "wp3"){
		    pose1.orientation.w = 0.70;
		    pose1.orientation.x = 0.00;
		    pose1.orientation.y = 0.00;
		    pose1.orientation.z = 1.4;
		    
		    pose1.position.x =  -3.0;
		    pose1.position.y =  0.00;
		    }
		else if (msg->parameters[2].value == "wp4"){
		    pose1.orientation.w = 0.70;
		    pose1.orientation.x = 0.00;
		    pose1.orientation.y = 0.00;
		    pose1.orientation.z = -1.4;
		    
		    pose1.position.x =  0.0;
		    pose1.position.y =  -3.0;
		    }
		if (msg->parameters[1].value == "h1"){
		    pose1.position.z =  0.75;
		    }
		if (msg->parameters[1].value == "h2"){
		    pose1.position.z =  1.25;
		    }*/
		/*
		group.setStartStateToCurrentState();
		group.setApproximateJointValueTarget(pose1, "cluedo_link");
		    
		std::vector<double> joint_values;
                double timeout = 0.5;
                bool found_ik = kinematic_state->setFromIK(joint_model_group, pose1, timeout);
  
                // Now, we can print out the IK solution (if found):
                if (found_ik)
                {
                   kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
                   for (std::size_t i = 0; i < joint_names.size(); ++i)
                   {
                       ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
                   }
    
               }
               else
               {
                   ROS_INFO("Did not find IK solution");
               }
  
               group.setJointValueTarget(joint_values);
               group.setStartStateToCurrentState();
               group.setGoalOrientationTolerance(0.01);
               group.setGoalPositionTolerance(0.01);

               // Plan and execute
                   
               group.plan(my_plan); 
               group.execute(my_plan);
	       */
	       sleep(5);
	       ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
	       return true;
	}
}

	int main(int argc, char **argv) {
		ros::init(argc, argv, "MoveArm_rosplan_action", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");
		
		ros::AsyncSpinner spinner(1);
		spinner.start();
		
  
		KCL_rosplan::MoveArmInterface my_aci(nh);
		my_aci.runActionInterface();
		return 0;
	}
