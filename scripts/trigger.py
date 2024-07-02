#! /usr/bin/env python
import rospy
from std_srvs.srv import Trigger, TriggerRequest
from sensor_msgs.msg import PointCloud2
import ros_numpy
import numpy as np

def callback_fn(msg: PointCloud2):
    pc = ros_numpy.numpify(msg)
    points=np.zeros((pc.shape[0]*pc.shape[1],3))
    points[:,0]=pc['x'].flatten()
    points[:,1]=pc['y'].flatten()
    points[:,2]=pc['z'].flatten()
    
    print("Received point cloud lengths")
    print(points[:,0].shape)
    print(points[:,1].shape)
    print(points[:,2].shape)
    print()
    
    pass
if __name__ == '__main__':
    rospy.init_node('test_trigger')
    
    phoxi_sub = rospy.Subscriber("/phoxi/pointcloud", PointCloud2, callback_fn)
    motion_sub = rospy.Subscriber("/motion/pointcloud", PointCloud2, callback_fn)
    
    rospy.wait_for_service('/phoxi/trigger_frame')
    rospy.wait_for_service('/motion/trigger_frame')
    phoxi_service = rospy.ServiceProxy('/phoxi/trigger_frame', Trigger)
    motion_service = rospy.ServiceProxy('/motion/trigger_frame', Trigger)
    
    frame_trigger = TriggerRequest()
    
    phoxi_service(frame_trigger)
    motion_service(frame_trigger)
    
    rospy.spin()