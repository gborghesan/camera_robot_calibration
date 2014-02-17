#!/usr/bin/env python  

import numpy as num
from geometry_msgs.msg import Pose, Point, Quaternion
from tf_conversions import posemath

import PyKDL

def load_pose_from_file(fname):
    with open(fname) as f:
        x, y, z, xx, yy, zz, ww =[float(x) for x in f.readline().split()]
        w_T_c_in=posemath.fromMsg(Pose(Point(x,y,z),Quaternion(xx,yy,zz,ww)))
        x, y, z, xx, yy, zz, ww =[float(x) for x in f.readline().split()]
        ee_T_m_in=posemath.fromMsg(Pose(Point(x,y,z),Quaternion(xx,yy,zz,ww)))
        array = []
        for line in f: # read rest of lines
            x, y, z, xx, yy, zz, ww =[float(x) for x in line.split()]
            array.append(posemath.fromMsg(Pose(Point(x,y,z),Quaternion(xx,yy,zz,ww))))

        w_T_ee_vec=[]
        c_T_m_vec=[]
        for i in range(0,len(array),2):

            w_T_ee_vec.append(array[i])
            c_T_m_vec.append(array[i+1])
        return (w_T_c_in,ee_T_m_in,w_T_ee_vec,c_T_m_vec)
        
            
    


def create_A_B(w_T_ee, ee_T_mn,w_T_cn,cr_T_mr):
    """
    Generates the A and B sub-matrices  for the optimal problem, for each set of poses
    
    Parameters
    ----------
    w_T_ee : geometry_msgs.Pose
        pose from the common frame 'w' to the link of the robot where the marker is fastened (end-effector), 
        read from the robot
    ee_T_mn : geometry_msgs.Pose
        nominal pose from end-effector to marker reference.
    w_T_cn : geometry_msgs.Pose
        nominal pose from common frame 'w' to camera
    cr_T_mr : geometry_msgs.Pose
        pose from camera frame to marker, read from the marker identification (e.g. ar_pose)
        
    """
    w_T_mn=w_T_ee*ee_T_mn
    A =num.matrix([[ -w_T_mn.M[0,0], -w_T_mn.M[0,1], -w_T_mn.M[0,2], cr_T_mr.p.y()*w_T_cn.M[0,2] - cr_T_mr.p.z()*w_T_cn.M[0,1], cr_T_mr.p.z()*w_T_cn.M[0,0] - cr_T_mr.p.x()*w_T_cn.M[0,2], cr_T_mr.p.x()*w_T_cn.M[0,1] - cr_T_mr.p.y()*w_T_cn.M[0,0], w_T_cn.M[0,0], w_T_cn.M[0,1], w_T_cn.M[0,2]],
                   [ -w_T_mn.M[1,0], -w_T_mn.M[1,1], -w_T_mn.M[1,2], cr_T_mr.p.y()*w_T_cn.M[1,2] - cr_T_mr.p.z()*w_T_cn.M[1,1], cr_T_mr.p.z()*w_T_cn.M[1,0] - cr_T_mr.p.x()*w_T_cn.M[1,2], cr_T_mr.p.x()*w_T_cn.M[1,1] - cr_T_mr.p.y()*w_T_cn.M[1,0], w_T_cn.M[1,0], w_T_cn.M[1,1], w_T_cn.M[1,2]],
                   [ -w_T_mn.M[2,0], -w_T_mn.M[2,1], -w_T_mn.M[2,2], cr_T_mr.p.y()*w_T_cn.M[2,2] - cr_T_mr.p.z()*w_T_cn.M[2,1], cr_T_mr.p.z()*w_T_cn.M[2,0] - cr_T_mr.p.x()*w_T_cn.M[2,2], cr_T_mr.p.x()*w_T_cn.M[2,1] - cr_T_mr.p.y()*w_T_cn.M[2,0], w_T_cn.M[2,0], w_T_cn.M[2,1], w_T_cn.M[2,2]]])
 
    B =num.matrix([[ w_T_mn.p.x() - w_T_cn.p.x() - cr_T_mr.p.x()*w_T_cn.M[0,0] - cr_T_mr.p.y()*w_T_cn.M[0,1] - cr_T_mr.p.z()*w_T_cn.M[0,2]],
                   [ w_T_mn.p.y() - w_T_cn.p.y() - cr_T_mr.p.x()*w_T_cn.M[1,0] - cr_T_mr.p.y()*w_T_cn.M[1,1] - cr_T_mr.p.z()*w_T_cn.M[1,2]],
                   [ w_T_mn.p.z() - w_T_cn.p.z() - cr_T_mr.p.x()*w_T_cn.M[2,0] - cr_T_mr.p.y()*w_T_cn.M[2,1] - cr_T_mr.p.z()*w_T_cn.M[2,2]]])             
    return [A,B]



class camera_robot_calibration():
    def __init__(self):

        # camera base in world
        self.w_T_c=PyKDL.Frame.Identity()
        # marker in ee
        self.ee_T_m=PyKDL.Frame.Identity()
        #vectors of saved data (kdl frames)
        self._w_T_ee=[]
        self._c_T_m=[]
        

    def store_frames(self,w_T_ee,c_T_m):
        """ 
        save frames read from camera and robot.
        Parameters
        ----------    
        w_T_ee : geometry_msgs.Pose
        pose from the common frame 'w' to the link of the robot where the marker is fastened (end-effector), 
        read from the robot
        c_T_m : geometry_msgs.Pose
        pose from camera frame to marker, read from the marker identification (e.g. ar_pose)."""
   
        self._w_T_ee.append(w_T_ee)
        self._c_T_m.append(c_T_m)
        
    def reset_frames(self):
        """throw away saved measured frames till now"""
        self._w_T_ee=[]
        self._c_T_m=[]
                
    def set_intial_frames(self,w_T_c,ee_T_m):
        """"
        Set initial estimates
        
        Parameters
        ----------
        ee_T_m : geometry_msgs.Pose
            nominal pose from end-effector to marker reference.
        w_T_c : geometry_msgs.Pose
            nominal pose from common frame 'w' to camera
        cr_T_mr : geometry_msgs.Pose
            pose from camera frame to marker, read from the marker identification (e.g. ar_pose)"""
        
        self.w_T_c=w_T_c        

        self.ee_T_m=ee_T_m
        
    def compute_frames(self,r_eq=1,w_marker=1):
        """"
        function that computes the frame of the camera (accessible as self.w_T_c) and
        the position of the marker w.r.t. the robot end_effector (self.ee_T_m)
        the inputs are normalizations factors to weight pseudo-inverse
        
        Parameters
        ----------
        r_eq : scalar
            equivalent radius of camera orientation.
            As rule of the thumb, you should choose this as more or less of the distance
            btw the marker registered positions and the camera.
        w_marker: scalar
            weights given to the marker position.

        """
            #compute the big matrix
        self.Atot=None
        self.Btot=None
        for i in range(len(self._w_T_ee)):
            [Asub,Bsub]=create_A_B(self._w_T_ee[i],self.ee_T_m,self.w_T_c,self._c_T_m[i])
            if self.Atot is None:
                self.Atot=Asub
                self.Btot=Bsub
            else:
                self.Atot=num.vstack([self.Atot, Asub])
                self.Btot=num.vstack([self.Btot, Bsub])
            
                
            #matrix are built, compute the weighted pseudoinverse
            #converting rotation with a 1 meter scale
            
            #the x is organized as follows:
        W=num.matrix(num.diag(num.array([1,1,1,r_eq,r_eq,r_eq,w_marker,w_marker,w_marker])))

        x=num.linalg.pinv(self.Atot*W)*self.Btot;
            
            #now i have to update the poses
            
        Delta_m=PyKDL.Frame(PyKDL.Rotation.Identity(),PyKDL.Vector(x[0],x[1],x[2]))
              
        self.ee_T_m=self.ee_T_m*Delta_m;
            
        Delta_c=PyKDL.Frame(PyKDL.Rotation.RPY(x[3],x[4],x[5]),
            PyKDL.Vector(x[6],x[7],x[8]))
            
        self.w_T_c=self.w_T_c*Delta_c;

            #compute the residue
        return self.Atot*x-self.Btot

            
            #

if __name__ == '__main__':
    
    #self test: it needs some functions that simulate the measurements,
    # like a generator of random ee poses
    import matplotlib.pyplot as plt
    random_poses =False
    crc=camera_robot_calibration()
    if (random_poses):
        from random import random
    
        def random_pose(angle_range=num.pi/4, pos_range=0.5):
            return PyKDL.Frame(PyKDL.Rotation.RPY(angle_range*random(),angle_range*random(),angle_range*random()),
                               PyKDL.Vector(pos_range*random(),pos_range*random(),pos_range*random()))
    
        #define real poses of camera and marker
        w_TR_c=random_pose(num.pi/2,0.5)
        ee_TR_m=random_pose(num.pi/2,0.5)
        #w_TR_c=PyKDL.Frame.Identity();
        #ee_TR_m=PyKDL.Frame.Identity();
        
        
    
       
        #generate some positions of the robot, and make some camera measurements
        w_T_ee_vec=[]
        c_T_m_vec=[]
        w_P_c_in=Pose();
        ee_P_m_in=Pose();
        for i in range(10):

            #generate robot position
            w_T_ee_vec.append(random_pose())
            #measure the frame of the marker w.r.t. the camera, plus some noise 
            #(noise on angle is not necessary as only marker position is used)
            c_T_m_vec.append(w_TR_c.Inverse()*w_T_ee*ee_TR_m*random_pose(0,0.01))

    else:
            #load data from a file
           (w_T_c_in,ee_T_m_in,w_T_ee_vec,c_T_m_vec)=load_pose_from_file('data1.txt')
            
        #store the frames...
    crc.set_intial_frames(w_T_c_in,ee_T_m_in)
    for i in range(0,len(w_T_ee_vec)):
  
        crc.store_frames(w_T_ee_vec[i],c_T_m_vec[i])
    
    residue_max=[]
    residue_mod=[]
    #residue_all=None
    #x_bar=None
    #y_bar=None
    n_comp=10
    for i in range(n_comp):
        print crc.w_T_c.p
        residue=crc.compute_frames();
        r2=residue.transpose()*residue
        residue_mod.append( num.sqrt (r2[0,0]))
        residue_max.append(num.max(residue))
  
            
    if random_poses:
        print 'error w_T_c'
        print (w_TR_c.Inverse()*crc.w_T_c)
        print 'error ee_T_m.p'
        print (ee_TR_m.p-crc.ee_T_m.p)    
    print 'residue, maxes for iterations'
    print residue_max
    import matplotlib.pyplot as plt
    
    plt.plot(range(1,n_comp+1),residue_mod,'bo',range(1,n_comp+1),residue_mod,'k')
    plt.xlim([0,n_comp+1])
    plt.plot(range(1,n_comp+1),residue_max,'ro',range(1,n_comp+1),residue_max,'r')
 #   from mpl_toolkits.mplot3d import Axes3D
  #  fig=plt.figure(2)
  #  ax=Axes3D(fig)
    plt.xlabel('iteration #')
    plt.grid() 
    print ('camera pose:'+str(crc.w_T_c))
    plt.show()
    
            

