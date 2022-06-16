#include <ros/ros.h>
#include "rosplan_action_interface/RPActionInterface.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


namespace KCL_rosplan {

	class AdjustInitHeightInterface: public RPActionInterface
	{

	private:

	public:

		/* constructor */
		AdjustInitHeightInterface(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}

