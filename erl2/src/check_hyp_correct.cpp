#include "erl2/check_hyp_correct.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <erl2/PlanningAction.h>
#include "erl2/HypCompCheck.h"
#include "erl2/Oracle.h"
#include <string.h>
#include <vector>
#include <typeinfo>

ros::ServiceClient client_ont;
ros::ServiceClient client_oracle;


namespace KCL_rosplan {

	CheckHypCorrectInterface::CheckHypCorrectInterface(ros::NodeHandle &nh) {
	    // here the initialization
	}

	bool CheckHypCorrectInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		// here the implementation of the action
		
		erl2::HypCompCheck h;    
		client_ont.call(h);
		ROS_INFO("I called the check complete hypothesis");
		//std::string complete_hypotheses[] = h.response.complete_hyps;
		//std::cout<<"Type of check complete response: "<<typeid(h.response.complete_hyps).name()<<std::endl;
		std::vector<std::string> r = h.response.complete_hyps;
		int length_hyp = r.size();
		if (length_hyp == 0){
		    ROS_INFO("No complete hypothesis yet");
		    return false;
		}
		else{
		    ROS_INFO("There are some complete hypotheses");
		    erl2::Oracle o;
		    client_oracle.call(o);
		    std::string correct_ID = std::to_string(o.response.ID);
		    std::cout<<"Correct hypothesis ID: "<<correct_ID<<std::endl;
		    std::cout<<"Current Complete Hypothesis IDs:"<<std::endl;
		    bool correct_flag = false;
		    for(int i=0; i<length_hyp; i++){
		        std::cout<<r[i]<<std::endl;
		        if (r[i] == correct_ID){
		            correct_flag = true;
		        }
		    }
		    if(correct_flag){
		        ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
		        return true;
		    }
		    else{
		        return false;
		    }
		}
		
		        
		
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "CheckHypCorrect_rosplan_action", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	client_ont = nh.serviceClient<erl2::HypCompCheck>("/check_hyp_complete");
	client_oracle = nh.serviceClient<erl2::Oracle>("/oracle_solution");
	KCL_rosplan::CheckHypCorrectInterface my_aci(nh);
	my_aci.runActionInterface();
	return 0;
}
