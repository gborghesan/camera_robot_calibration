#!/usr/bin/env python  
"""
Copyright (c) 2014, Gianni Borghesan 
All rights reserved. 

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are met: 

 * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer. 
 * Redistributions in binary form must reproduce the above copyright 
   notice, this list of conditions and the following disclaimer in the 
   documentation and/or other materials provided with the distribution. 
 * Neither the name of KU Leuven nor the names of its contributors may be 
   used to endorse or promote products derived from this software without 
   specific prior written permission. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
POSSIBILITY OF SUCH DAMAGE. 
"""

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
    Generates the A and B sub-matrices  for the optimization problem, for each set of poses
    
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
    def usage():
        print "Usage: \n\tcamera_robot_calibration_module [-h][-i file_of_input][-p True/False]"
    import sys, getopt

    try:
       opts, args = getopt.getopt(sys.argv[1:], "hi:p:", ["help", "inputfile=", "plotgraph="])
       print opts
    except getopt.GetoptError as err:
        # print help information and exit:
        print str(err) # will print something like "option -a not recognized"
        usage()
        sys.exit(2)
        
    plot_grap=True
    inputfile=None

    for o, a in (opts):
        if o in("-i","--inputfile"):
            inputfile = a
        elif o in ("-h", "--help"):
            usage()
            sys.exit()
        elif o in ("-p", "--plotgraph"):
            print a
            if a=='True': 
                plot_grap=True
            elif a=='False': 
                plot_grap=False
            else:
                usage()
                sys.exit()
        else:
            assert False, "unhandled options"
    

    
    #self test: it needs some functions that simulate the measurements,
    # like a generator of random ee poses

    if inputfile==None:
        random_poses =True
    else:
         random_poses =False
    crc=camera_robot_calibration()
    
    if (random_poses):
        from random import random
        noise =0.01
        def random_pose(angle_range=num.pi/4, pos_range=0.5):
            return PyKDL.Frame(PyKDL.Rotation.RPY(angle_range*random(),angle_range*random(),angle_range*random()),
                               PyKDL.Vector(pos_range*random(),pos_range*random(),pos_range*random()))
    
        #define real poses of camera and marker
        w_TR_c=random_pose(num.pi/2,1)
        ee_TR_m=random_pose(num.pi/2,0.1)

        #generate some positions of the robot, and make some camera measurements
        w_T_ee_vec=[]
        c_T_m_vec=[]
        w_T_c_in=random_pose()
        ee_T_m_in=random_pose()
        w_P_c_in=Pose();
        ee_P_m_in=Pose();
        for i in range(10):
            #generate robot position
            w_T_ee=random_pose()
            w_T_ee_vec.append(w_T_ee)
            #measure the frame of the marker w.r.t. the camera, plus some noise 
            #(noise on angle is not necessary as only marker position is used)
            c_T_m_vec.append(w_TR_c.Inverse()*w_T_ee*ee_TR_m*random_pose(0,noise))
    else:
        #measurements form file are used
        (w_T_c_in,ee_T_m_in,w_T_ee_vec,c_T_m_vec)=load_pose_from_file(inputfile)
            
        #store the frames...
    crc.set_intial_frames(w_T_c_in,ee_T_m_in)
    for i in range(0,len(w_T_ee_vec)):
        crc.store_frames(w_T_ee_vec[i],c_T_m_vec[i])
    
    residue_max=[]
    residue_average=[]

    n_comp=4
    for i in range(n_comp):
        print crc.w_T_c.p
        residue=crc.compute_frames();
        residue_average.append( num.average(num.abs(residue)))
        residue_max.append(num.max(num.abs(residue)))

  
            
    if random_poses:
        print 'noise measurement ([x,y,z] marker position, in meters)'
        print noise
        print 'error camera pose: computed vs Ground Truth'
        print (w_TR_c.Inverse()*crc.w_T_c)
        print 'error marker position: computed vs Ground Truth'
        print (ee_TR_m.p-crc.ee_T_m.p)    
    print 'residue, maxes for iterations'
    print residue_max
    if plot_grap==True:
        import matplotlib.pyplot as plt
        ax = plt.subplot(1,1,1)
        ax.plot(range(1,n_comp+1),residue_average,'bo:', label="average residue")
        plt.xlim([0.8,n_comp+.2])
        ax.plot(range(1,n_comp+1),residue_max,'ro:', label="max residue") 
        #ax.set_yscale('log')   
        handles, labels = ax.get_legend_handles_labels()
        plt.legend(handles, labels)   

        plt.xlabel('iteration #')
        plt.ylabel('residue (meters)')
        plt.grid() 
        print ('camera pose:\n'+str(crc.w_T_c))
        [R, P, Y] = crc.w_T_c.M.GetRPY()
        print ('Yaw:\t'+str(Y)+'\nPitch:\t'+str(P)+'\nRoll:\t'+str(R))
        plt.show()
        