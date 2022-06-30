/**
 * \file check_hyp_correct.cpp
 * \brief this file is an implementation of the check hypothesis correct PDDL action
 * \author Yara Abdelmottaleb
 * \version 0.1
 * \date 30/06/2022
 *
 * \details
 *
 * Clients: <BR>
 *   /check_hyp_complete
 *  
 *   /oracle_solution
 *
 * Description :
 *
 * This node implements the interface for the PDDL action check hypothesis correct
 * It checks whether the any of the current completed hypotheses in the ontology is the correct hypothesis or not
 * 
 *
 *
*/

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
        /**
	 * \brief this is the initialization function of CheckHypCorrectInterface class
	 * This function takes as input the node handle and initiliazes an instance of the class
	*/
	CheckHypCorrectInterface::CheckHypCorrectInterface(ros::NodeHandle &nh) {
	    // here the initialization
	}
        
	/**
	 * \brief this is the callback function for the check hypothesis correct action interface
	 * 
	 *
	 * \return true if there is a correct hypothesis or false otherwise
	 *
	 * This function gets the list of completed hypotheses by calling the service /check_hyp_complete and then it gets the correct hypothesis ID by calling the service /oracle_solution 
	 * It loops over the list of completed hypotheses to check if any of them is the correct hypothesis
	 * 
	*/
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

/**
 * \brief this is main function of the node
 * It initializes the node handle and the action interface, 
 *
*/
int main(int argc, char **argv) {
	ros::init(argc, argv, "CheckHypCorrect_rosplan_action", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	client_ont = nh.serviceClient<erl2::HypCompCheck>("/check_hyp_complete");
	client_oracle = nh.serviceClient<erl2::Oracle>("/oracle_solution");
	KCL_rosplan::CheckHypCorrectInterface my_aci(nh);
	my_aci.runActionInterface();
	return 0;
}
