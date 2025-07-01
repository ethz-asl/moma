#!/usr/bin/env python

import rospy
import sys
sys.path.insert(0,'/root/moma_ws/src/moma/moma_demos/stacking_demo/src/stacking_demo/')

from realworld_stacking_pose_predictor import  run_realworld_stacking_pose_predictor

from stacking_demo.srv import GetTowerPrediction, GetTowerPredictionRequest, GetTowerPredictionResponse

def get_tower_prediction(req: GetTowerPredictionRequest) -> GetTowerPredictionResponse: 
    """wait for model service handler"""
    rospy.loginfo("Waiting for model to predict tower position")
    
    obj_id = req.obj_id 
    y_offset = run_realworld_stacking_pose_predictor(obj_id)

    return GetTowerPredictionResponse(y_offset)

if __name__ == "__main__":
    rospy.init_node("get_tower_prediction_node", log_level=rospy.INFO)
    
    service = rospy.Service("get_tower_prediction", GetTowerPrediction, get_tower_prediction)
    rospy.spin()
