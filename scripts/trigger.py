#! /usr/bin/env python
import rospy
from std_srvs.srv import Trigger, TriggerRequest

if __name__ == '__main__':
    rospy.init_node('test_trigger')
    rospy.wait_for_service('/phoxi/trigger_frame')
    trigger_service = rospy.ServiceProxy('/phoxi/trigger_frame', Trigger)
    
    frame_trigger = TriggerRequest()
    
    res = trigger_service(frame_trigger)
    print(res)