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
def create_A_B(w_P_ee, ee_P_mn,w_P_cn,cr_P_mr):
#inputs: poses of the ee
#        poses marker nominal w.r.t to ee
#        poses nominla of the camera
#        poses measured by the camera of the marker
    w_T_ee=pose_to_matrix(w_P_ee)
    ee_T_mn=pose_to_matrix(ee_P_mn)
    w_T_mn=w_T_ee*ee_T_mn
    w_p_mn=w_T_mn[0:3,3]
    w_R_mn=w_T_mn[0:3,0:3]
    w_T_cn=pose_to_matrix(w_P_cn)
    w_p_cn=w_T_cn[0:3,3]
    w_R_cn=w_T_cn[0:3,0:3]
    cr_T_mr=pose_to_matrix(cr_P_mr)
    cr_p_mr=cr_T_mr[0:3,3]
    A =num.matrix([[ -w_R_mn[0,0], -w_R_mn[0,1], -w_R_mn[0,2], cr_p_mr[1,0]*w_R_cn[0,2] - cr_p_mr[2,0]*w_R_cn[0,1], cr_p_mr[2,0]*w_R_cn[0,0] - cr_p_mr[0,0]*w_R_cn[0,2], cr_p_mr[0,0]*w_R_cn[0,1] - cr_p_mr[1,0]*w_R_cn[0,0], w_R_cn[0,0], w_R_cn[0,1], w_R_cn[0,2]],
                   [ -w_R_mn[1,0], -w_R_mn[1,1], -w_R_mn[1,2], cr_p_mr[1,0]*w_R_cn[1,2] - cr_p_mr[2,0]*w_R_cn[1,1], cr_p_mr[2,0]*w_R_cn[1,0] - cr_p_mr[0,0]*w_R_cn[1,2], cr_p_mr[0,0]*w_R_cn[1,1] - cr_p_mr[1,0]*w_R_cn[1,0], w_R_cn[1,0], w_R_cn[1,1], w_R_cn[1,2]],
                   [ -w_R_mn[2,0], -w_R_mn[2,1], -w_R_mn[2,2], cr_p_mr[1,0]*w_R_cn[2,2] - cr_p_mr[2,0]*w_R_cn[2,1], cr_p_mr[2,0]*w_R_cn[2,0] - cr_p_mr[0,0]*w_R_cn[2,2], cr_p_mr[0,0]*w_R_cn[2,2] - cr_p_mr[1,0]*w_R_cn[2,0], w_R_cn[2,0], w_R_cn[2,1], w_R_cn[2,2]]])
    #print (A)
    B =num.matrix([[ w_p_mn[0,0] - w_p_cn[0,0] - cr_p_mr[0,0]*w_R_cn[0,0] - cr_p_mr[1,0]*w_R_cn[0,1] - cr_p_mr[2,0]*w_R_cn[0,2]],
                   [ w_p_mn[1,0] - w_p_cn[1,0] - cr_p_mr[0,0]*w_R_cn[1,0] - cr_p_mr[1,0]*w_R_cn[1,1] - cr_p_mr[2,0]*w_R_cn[1,2]],
                   [ w_p_mn[2,0] - w_p_cn[2,0] - cr_p_mr[0,0]*w_R_cn[2,0] - cr_p_mr[1,0]*w_R_cn[2,1] - cr_p_mr[2,0]*w_R_cn[2,2]]])             
    #print (B)
    return [A,B]

def pose_to_matrix(pose_in):
    vtr = (pose_in.position.x, pose_in.position.y,
            pose_in.position.z)
    vrot = (pose_in.orientation.x, pose_in.orientation.y, pose_in.orientation.z,
            pose_in.orientation.w)
    return num.matrix(tf.TransformerROS().fromTranslationRotation(vtr, vrot))

def matrix_to_pose(m):
    p=Pose();
    p.position.x=m[0,3]
    p.position.y=m[1,3]
    p.position.z=m[2,3]
    q=tf.transformations.quaternion_from_matrix(tuple(map(tuple,m.A)))
    p.orientation.w=q[3]
    p.orientation.x=q[0]
    p.orientation.y=q[1]
    p.orientation.z=q[2]  
    return p

class camera_robot_calibration():
    def __init__(self):
        #read values from properties
        self.base_frame_name=rospy.get_param('base_frame_name', '/base_link')
        self.camera_frame_name=rospy.get_param('camera_frame_name', '/camera_link')
        self.robot_ee_frame_name=rospy.get_param('robot_ee_frame_name', '/lwr_arm_link_7')
        self.target_frame_name=rospy.get_param('target_frame_name', '/marker_frame')
       

        #nominal positions of camera w.r.t world and marker mounted in the robot
        #this two frames are published
        unity_frame=Pose()
        unity_frame.orientation.w=1; 
	    # marker in ee
        self.ee_P_m=rospy.get_param('robot_ee_pose_camera', unity_frame);
        # camera base in world
        unity_frame.position.z=0.5
        self.w_P_c=rospy.get_param('nominal_pose_camera', unity_frame);
        
        
        #setup TF LISTENER AND BROADCASTER
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        
        #vectors of saved data
        self.w_P_ee=[]
        self.c_P_m=[]
        
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
        # if i read both i save in corrsponding vectors
        if ok:
            self.w_P_ee.append(w_P_ee)
            self.c_P_m.append(c_P_m)
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
                         self.base_frame_name,
                         self.target_frame_name+"_nominal")
    
        
    def compute_frames(self):
            #compute the big matrix
            Atot=None
            Btot=None
            for i in range(len(self.w_P_ee)):
               [Asub,Bsub]=create_A_B(self.w_P_ee[i],self.ee_P_m,self.w_P_c,self.c_P_m[i])
               if Atot is None:
                   Atot=Asub
                   Btot=Bsub
               else:
                   Atot=num.hstack([Atot, Asub])
                   Btot=num.hstack([Btot, Bsub])
            #now i have the matrix, compute the weighted pseudoinverse
            r_eq=1 #converting rotation with a 1 meter scale
            Wy=num.matrix(num.diag(num.array([1,1,1,r_eq,r_eq,r_eq,1,1,1])))
            x=num.linalg.pinv(Atot*Wy)*Btot;
            
            #now i have to update the poses
            #ee_T_mn=ee_T_mn*[eye(3),x(1:3);[0 0 0 1]];
            #w_T_cn=w_T_cn*[rpy2r(x(4:6)'),x(7:9);[0 0 0 1]];
            
            Delta_m=PyKDL.Frame(PyKDL.Rotation.Identity(),PyKDL.Vector(x[0],x[1],x[2]))
            ee_T_m=posemath.fromMsg(self.ee_P_m)   
            ee_T_m=ee_T_m*Delta_m;
            self.ee_P_m = PyKDL.toMsg(ee_T_m)
            
            Delta_c=PyKDL.Frame(PyKDL.Rotation.RPY(x[3],x[4],x[5]),
                PyKDL.Vector(x[6],x[7],x[8]))
            w_T_m=posemath.fromMsg(self.w_P_c)
            w_T_m=w_T_m*Delta_c;
            self.w_P_c=PyKDL.toMsg(w_T_m)
                
            
            #

if __name__ == '__main__':
    rospy.init_node('camera_robot_calibration')
    est=camera_robot_calibration()
    
    while not rospy.is_shutdown():
      est.publish_tfs()
      rospy.sleep(0.01)

    rospy.spin()
