#! /usr/bin/env python
import rospy
from std_srvs.srv import Trigger, TriggerRequest
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import open3d as o3d

def callback_fn(msg: PointCloud2):
    pc = np.array(list(pc2.read_points(msg, field_names=("x","y","z","rgb","normal_x","normal_y","normal_z"),skip_nans=True)))
    points = np.zeros((pc.shape[0],3))
    normals= np.zeros((pc.shape[0],3))
    
    points[:,0]=pc[:,0]
    points[:,1]=pc[:,1]
    points[:,2]=pc[:,2]
    normals[:,0]=pc[:,4]
    normals[:,1]=pc[:,5]
    normals[:,2]=pc[:,6]
    
    print("Received point cloud lengths")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.normals = o3d.utility.Vector3dVector(normals)
    o3d.visualization.draw_geometries([pcd])
    
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
    rospy.sleep(3)
    motion_service(frame_trigger)
    
    rospy.spin()