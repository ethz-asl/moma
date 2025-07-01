#!/usr/bin/env python

import rospy


from stacking_demo.srv import GetTowerPrediction, GetTowerPredictionRequest, GetTowerPredictionResponse

def get_tower_prediction(req: GetTowerPredictionRequest) -> GetTowerPredictionResponse: 
    """wait for model service handler"""
    rospy.loginfo("Waiting for model to predict tower position")
    
    ## add logic to wait here for model
    # PW TODO

    # also set the tower position in rosparam
    # rospy.set_param("tower_offset_y", 0.0)  
    rospy.sleep(5) # simulating waiting time
    y_offset =0.05

    return GetTowerPredictionResponse(y_offset)

if __name__ == "__main__":
    rospy.init_node("get_tower_prediction_node", log_level=rospy.INFO)
    
    service = rospy.Service("get_tower_prediction", GetTowerPrediction, get_tower_prediction)
    rospy.spin()
