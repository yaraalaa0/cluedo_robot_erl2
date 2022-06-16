import roslib
import rospy
from armor_api.armor_client import ArmorClient
from erl2.srv import Hint
from erl2.srv import HypCompCheck, HypCompCheckResponse

path = "/root/Desktop/" #the path to cluedo ontology (.owl) file
ontology_IRI = "http://www.emarolab.it/cluedo-ontology"


def add_hint(req):
    global armor_client
    print("Ontology SERVER Received Request")
    #get the list of current complete hypotheses
    all_hyp_links = armor_client.query.ind_b2_class("HYPOTHESIS")
    all_hyp = [x.replace("<"+ontology_IRI+"#", '').replace('>','') for x in all_hyp_links]
    print(all_hyp)
    if req.ID not in all_hyp:
        print("Added a new hypothesis ID")
        #add it to the hypothesis class
        armor_client.manipulation.add_ind_to_class(req.ID, "HYPOTHESIS")
        armor_client.manipulation.add_dataprop_to_ind("hasID", req.ID, "STRING", req.ID)
    
    #add the hint to the current hypothesis object properties
    if req.key == "what":
        armor_client.manipulation.add_ind_to_class(req.value, "WEAPON")
        armor_client.manipulation.add_objectprop_to_ind(req.key, req.ID, req.value)
    
    elif req.key == "who":
        armor_client.manipulation.add_ind_to_class(req.value, "PERSON")
        armor_client.manipulation.add_objectprop_to_ind(req.key, req.ID, req.value)
    
    elif req.key == "where":
        armor_client.manipulation.add_ind_to_class(req.value, "PLACE")
        armor_client.manipulation.add_objectprop_to_ind(req.key, req.ID, req.value)
    
    #apply the changes
    armor_client.utils.apply_buffered_changes()
    armor_client.utils.sync_buffered_reasoner()
    armor_client.utils.save_ref_with_inferences(path + "erl2_cluedo.owl")
    return True
    

def check_complete(req):
    global armor_client
    #get the list of current complete hypotheses
    complete_hyp_links = armor_client.query.ind_b2_class("COMPLETED")
    complete_hyp = [x.replace("<"+ontology_IRI+"#", '').replace('>','') for x in complete_hyp_links]
    print("Server Complete Hypotheses:")
    print(complete_hyp)
    return HypCompCheckResponse(complete_hyp)
    '''
    if len(complete_hyp) == 0:
        return False
    else:
        return True
    '''
def main():
    global armor_client
    
    rospy.init_node('ontology_server')
    
    # Start ARMOR client and load the cluedo ontology
    armor_client = ArmorClient("client", "reference")
    armor_client.utils.load_ref_from_file(path + "erl2_cluedo.owl", ontology_IRI,
                                True, "PELLET", True, False)  # initializing with buffered manipulation and reasoning
    armor_client.utils.mount_on_ref()
    armor_client.utils.set_log_to_terminal(True)
    
    hint_service = rospy.Service('/add_hint', Hint, add_hint)
    check_comp_service = rospy.Service('/check_hyp_complete', HypCompCheck, check_complete)
    
    rospy.spin()



if __name__ == '__main__':
    main()
