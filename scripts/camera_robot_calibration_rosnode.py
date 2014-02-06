#!/usr/bin/env python  
import roslib
roslib.load_manifest('camera_robot_calibration')
import rospy
import tf
import numpy as num
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from tf_conversions import posemath
import PyKDL

from std_srvs.srv import Empty, EmptyResponse

from camera_robot_calibration_module import camera_robot_calibration

class camera_robot_calibration_ros():
    def __init__(self):
        #read values from properties
        self.base_frame_name=rospy.get_param('base_frame_name', '/base_link')
        self.camera_frame_name=rospy.get_param('camera_frame_name', '/camera_frame')
        self.robot_ee_frame_name=rospy.get_param('robot_ee_frame_name', '/ee_frame')
        self.target_frame_name=rospy.get_param('target_frame_name', '/marker_frame')
        
        #nominal positions of camera w.r.t world and marker mounted in the robot
        #this two frames are published
        unity_frame=Pose()
        unity_frame.orientation.w=1;
        # camera base in world
        self.w_P_c=rospy.get_param('nominal_pose_camera', unity_frame);
        # marker in ee
        self.ee_P_m=rospy.get_param('robot_ee_pose_camera', unity_frame);
        
        #setup TF LISTENER AND BROADCASTER
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        
        #vectors of saved data
        self.crc=camera_robot_calibration()
        
        #create services
        self.s1 = rospy.Service('read_tfs', Empty, self.read_tfs)
        self.s2 = rospy.Service('compute_frames', Empty, self.compute_frames)
        
    def read_tfs(self,req):
        #marker w.r.t. camera
        ok=True
        try:
           c_P_m = self.listener.lookupTransform(self.camera_frame_name,
                                                         self.target_frame_name, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            ok=False
         #ee w.r.t. base
        try:
           w_P_ee = self.listener.lookupTransform(self.base_frame_name,
                                                         self.robot_ee_frame_name, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            ok=False
        # if i read both i save in corresponding vectors
        if ok:
            self.crc.store_frames(posemath.fromMsg( w_P_ee),posemath.fromMsg(c_P_m))
            print b_P_ee
            print c_P_m
        else:
            print "error"
            
        return EmptyResponse();


    def publish_tfs(self):
        #publish the estimated poses of marker and camera, in tf
       
        self.br.sendTransform((self.w_P_c.position.x,self.w_P_c.position.y,self.w_P_c.position.z),  
                         (self.w_P_c.orientation.w,self.w_P_c.orientation.x,self.w_P_c.orientation.y,self.w_P_c.orientation.z),
                         rospy.Time.now(),
                         self.camera_frame_name,
                         self.base_frame_name)
        
        self.br.sendTransform((self.ee_P_m.position.x,self.ee_P_m.position.y,self.ee_P_m.position.z),  
                         (self.ee_P_m.orientation.w,self.ee_P_m.orientation.x,self.ee_P_m.orientation.y,self.ee_P_m.orientation.z),
                         rospy.Time.now(),
                         self.robot_ee_frame_name,
                         self.base_frame_name)
    
        
    def compute_frames(self):
            #read nominal poses, and set as initial positions
    
            self.crc.set_intial_frames(posemath.fromMsg( self.w_P_c),
                                        posemath.fromMsg(self.ee_P_m))

            
            #do several iteration of estimation
            n_comp=10
            residue_max=[]
            residue_mod=[]
            for i in range(n_comp):
                residue=crc.compute_frames();
                r2=residue.transpose()*residue
                residue_mod.append( num.sqrt (r2[0,0]))
                residue_max.append(num.max(residue))
            print 'residue_mod'
            print residue_mod
            print 'residue_max'
            print residue_max
            #put result back in parameter
            self.ee_P_m = PyKDL.toMsg(ee_T_m)
            self.w_P_c=PyKDL.toMsg(w_T_m)
                
            
            #

if __name__ == '__main__':
    rospy.init_node('camera_robot_calibration')
    est=camera_robot_calibration_ros()
    
    while not rospy.is_shutdown():
      est.publish_tfs()
      rospy.sleep(1.0)

    rospy.spin()
