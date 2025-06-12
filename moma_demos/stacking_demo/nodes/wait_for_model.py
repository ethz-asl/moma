#!/usr/bin/env python

import rospy


from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

def wait_for_model(req: TriggerRequest) -> TriggerResponse: 
    """wait for model service handler"""
    rospy.loginfo("Waiting for model to predict tower position")
    
    ## add logic to wait here for model

    # also set the tower position in rosparam
    rospy.set_param("tower_offset_y", 0.0)  
    rospy.sleep(10) # simulating waiting time

    return TriggerResponse(success=True, message="Model prediction completed.")

if __name__ == "__main__":
    rospy.init_node("wait_for_model_node", log_level=rospy.INFO)
    
    service = rospy.Service("wait_for_model", Trigger, wait_for_model)
    rospy.spin()
