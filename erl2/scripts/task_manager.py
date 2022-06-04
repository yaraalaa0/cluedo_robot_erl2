#!/usr/bin/env python

import roslib
import rospy
from std_srvs.srv import Empty, EmptyRequest
from rosplan_dispatch_msgs.srv import DispatchService, DispatchServiceRequest, DispatchServiceResponse
from rosplan_dispatch_msgs.msg import ActionDispatch
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService, KnowledgeUpdateServiceRequest
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArray, KnowledgeUpdateServiceArrayRequest
from diagnostic_msgs.msg import KeyValue


last_dispatched_action = "" # a variable to store the last dispatched action name to recognize the action that caused the plan failure
hints_received = 0 # a variable to store the number of hints received (because there is a problem with the hints function in PDDL, the hints is reset to 0 every time the get_hint action fails)

def clbk_ac_dispatch(msg):
    global last_dispatched_action, hints_received
    last_dispatched_action = msg.name
    if msg.name == 'get_hint':
        hints_received = hints_received + 1

def main():
    global last_dispatched_action, hints_received
    
    # initialize the ROS node
    rospy.init_node('task_manager')
                    
    
    # Initialize the rosplan services clients
    problem_client = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server',Empty)
    planning_server_client = rospy.ServiceProxy('/rosplan_planner_interface/planning_server',Empty)
    parser_client = rospy.ServiceProxy('/rosplan_parsing_interface/parse_plan',Empty)
    dispatcher_client = rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan',DispatchService)
    
    update_client = rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)
    arr_update_client = rospy.ServiceProxy('/rosplan_knowledge_base/update_array', KnowledgeUpdateServiceArray)
    action_dispatch_sub = rospy.Subscriber('/rosplan_plan_dispatcher/action_dispatch',ActionDispatch, clbk_ac_dispatch)
    
    # Call services 
    empty_req = EmptyRequest()
    problem_client(empty_req)
    planning_server_client(empty_req)
    parser_client(empty_req)
    req = DispatchServiceRequest()
    goal_success = dispatcher_client(req)
    print("Goal Success Result:")
    print(goal_success.success)
    print("Goal Achievement Result:")
    print(goal_success.goal_achieved)
    
    
    
    while (goal_success.success == False):
        if last_dispatched_action == "get_hint":
            print("get_hint action failed!")
            print("Re-Planning")
            hints_received = hints_received - 1
            print("Hints received")
            print(hints_received)
            
            # update the number of correct hints received to the hints function
            upd_req = KnowledgeUpdateServiceRequest()
            upd_req.update_type = 0
            upd_req.knowledge.knowledge_type = 2
            upd_req.knowledge.attribute_name = "hints"
            upd_req.knowledge.function_value = hints_received
            update_client(upd_req)
    
            # re-plan
            problem_client(empty_req)
            planning_server_client(empty_req)
            parser_client(empty_req)
            req = DispatchServiceRequest()
            goal_success = dispatcher_client(req)
            print("Goal Success Result:")
            print(goal_success.success)
            print("Goal Achievement Result:")
            print(goal_success.goal_achieved)
            
            
        elif last_dispatched_action == "check_hypothesis_complete" or last_dispatched_action == "check_hypothesis_correct":
            print("check_hypothesis action failed!")
            
            # update the current state of the system
            # reset all waypoints to unexplored
            upd_req = KnowledgeUpdateServiceRequest()
            upd_req.update_type = 0
            upd_req.knowledge.knowledge_type = 1
            upd_req.knowledge.attribute_name = "unexplored"
            v1 = KeyValue()
            v1.key = "waypoint"
            v1.value = "wp1"
            v2 = KeyValue()
            v2.key = "height"
            v2.value = "h1"
            upd_req.knowledge.values = [v1, v2]
            update_client(upd_req)
            
            upd_req.update_type = 0
            upd_req.knowledge.knowledge_type = 1
            upd_req.knowledge.attribute_name = "unexplored"
            upd_req.knowledge.values[0].key = "waypoint"
            upd_req.knowledge.values[0].value = "wp1"
            upd_req.knowledge.values[1].key = "height"
            upd_req.knowledge.values[1].value = "h2"
            update_client(upd_req)
            
            upd_req.update_type = 0
            upd_req.knowledge.knowledge_type = 1
            upd_req.knowledge.attribute_name = "unexplored"
            upd_req.knowledge.values[0].key = "waypoint"
            upd_req.knowledge.values[0].value = "wp2"
            upd_req.knowledge.values[1].key = "height"
            upd_req.knowledge.values[1].value = "h1"
            update_client(upd_req)
            
            upd_req.update_type = 0
            upd_req.knowledge.knowledge_type = 1
            upd_req.knowledge.attribute_name = "unexplored"
            upd_req.knowledge.values[0].key = "waypoint"
            upd_req.knowledge.values[0].value = "wp2"
            upd_req.knowledge.values[1].key = "height"
            upd_req.knowledge.values[1].value = "h2"
            update_client(upd_req)
            
            upd_req.update_type = 0
            upd_req.knowledge.knowledge_type = 1
            upd_req.knowledge.attribute_name = "unexplored"
            upd_req.knowledge.values[0].key = "waypoint"
            upd_req.knowledge.values[0].value = "wp3"
            upd_req.knowledge.values[1].key = "height"
            upd_req.knowledge.values[1].value = "h1"
            update_client(upd_req)
            
            upd_req.update_type = 0
            upd_req.knowledge.knowledge_type = 1
            upd_req.knowledge.attribute_name = "unexplored"
            upd_req.knowledge.values[0].key = "waypoint"
            upd_req.knowledge.values[0].value = "wp3"
            upd_req.knowledge.values[1].key = "height"
            upd_req.knowledge.values[1].value = "h2"
            update_client(upd_req)
            
            upd_req.update_type = 0
            upd_req.knowledge.knowledge_type = 1
            upd_req.knowledge.attribute_name = "unexplored"
            upd_req.knowledge.values[0].key = "waypoint"
            upd_req.knowledge.values[0].value = "wp4"
            upd_req.knowledge.values[1].key = "height"
            upd_req.knowledge.values[1].value = "h1"
            update_client(upd_req)
            
            upd_req.update_type = 0
            upd_req.knowledge.knowledge_type = 1
            upd_req.knowledge.attribute_name = "unexplored"
            upd_req.knowledge.values[0].key = "waypoint"
            upd_req.knowledge.values[0].value = "wp4"
            upd_req.knowledge.values[1].key = "height"
            upd_req.knowledge.values[1].value = "h2"
            update_client(upd_req)
            
            upd_req = KnowledgeUpdateServiceRequest()
            upd_req.update_type = 2
            upd_req.knowledge.knowledge_type = 1
            upd_req.knowledge.attribute_name = "complete_hyp"
            update_client(upd_req)
            
            upd_req = KnowledgeUpdateServiceRequest()
            upd_req.update_type = 2
            upd_req.knowledge.knowledge_type = 1
            upd_req.knowledge.attribute_name = "correct_hyp"
            update_client(upd_req)
            
            # reset the hints received to zero
            hints_received = 0
            
            # re-plan
            problem_client(empty_req)
            planning_server_client(empty_req)
            parser_client(empty_req)
            
            req = DispatchServiceRequest()
            goal_success = dispatcher_client(req)
            print("Goal Success Result:")
            print(goal_success.success)
            print("Goal Achievement Result:")
            print(goal_success.goal_achieved)
            
    
if __name__ == '__main__':
    main()
