#!/usr/bin/env python  

import numpy as num



import PyKDL

def create_A_B(w_T_ee, ee_T_mn,w_T_cn,cr_T_mr):
#inputs: poses of the ee
#        poses marker nominal w.r.t to ee
#        poses nominal of the camera
#        poses measured by the camera of the marker


    w_T_mn=w_T_ee*ee_T_mn
 
#[                   A =
    A =num.matrix([[ -w_T_mn.M[0,0], -w_T_mn.M[0,1], -w_T_mn.M[0,2], cr_T_mr.p.y()*w_T_cn.M[0,2] - cr_T_mr.p.z()*w_T_cn.M[0,1], cr_T_mr.p.z()*w_T_cn.M[0,0] - cr_T_mr.p.x()*w_T_cn.M[0,2], cr_T_mr.p.x()*w_T_cn.M[0,1] - cr_T_mr.p.y()*w_T_cn.M[0,0], w_T_cn.M[0,0], w_T_cn.M[0,1], w_T_cn.M[0,2]],
                   [ -w_T_mn.M[1,0], -w_T_mn.M[1,1], -w_T_mn.M[1,2], cr_T_mr.p.y()*w_T_cn.M[1,2] - cr_T_mr.p.z()*w_T_cn.M[1,1], cr_T_mr.p.z()*w_T_cn.M[1,0] - cr_T_mr.p.x()*w_T_cn.M[1,2], cr_T_mr.p.x()*w_T_cn.M[1,1] - cr_T_mr.p.y()*w_T_cn.M[1,0], w_T_cn.M[1,0], w_T_cn.M[1,1], w_T_cn.M[1,2]],
                   [ -w_T_mn.M[2,0], -w_T_mn.M[2,1], -w_T_mn.M[2,2], cr_T_mr.p.y()*w_T_cn.M[2,2] - cr_T_mr.p.z()*w_T_cn.M[2,1], cr_T_mr.p.z()*w_T_cn.M[2,0] - cr_T_mr.p.x()*w_T_cn.M[2,2], cr_T_mr.p.x()*w_T_cn.M[2,1] - cr_T_mr.p.y()*w_T_cn.M[2,0], w_T_cn.M[2,0], w_T_cn.M[2,1], w_T_cn.M[2,2]]])
    #print (A)
             #       w_p_mn_1_1 -   w_p_cn_1_1 -    cr_p_mr_1_1 *w_R_cn_1_1    - cr_p_mr_2_1    *w_R_cn_1_2  - cr_p_mr_3_1   * w_R_cn_1_3
#                    w_p_mn_2_1 -   w_p_cn_2_1 -    cr_p_mr_1_1 *w_R_cn_2_1    - cr_p_mr_2_1    *w_R_cn_2_2 -  cr_p_mr_3_1   * w_R_cn_2_3
 #                   w_p_mn_3_1 -   w_p_cn_3_1 -    cr_p_mr_1_1 *w_R_cn_3_1    - cr_p_mr_2_1    *w_R_cn_3_2  - cr_p_mr_3_1   * w_R_cn_3_3
    B =num.matrix([[ w_T_mn.p.x() - w_T_cn.p.x() - cr_T_mr.p.x()*w_T_cn.M[0,0] - cr_T_mr.p.y()*w_T_cn.M[0,1] - cr_T_mr.p.z()*w_T_cn.M[0,2]],
                   [ w_T_mn.p.y() - w_T_cn.p.y() - cr_T_mr.p.x()*w_T_cn.M[1,0] - cr_T_mr.p.y()*w_T_cn.M[1,1] - cr_T_mr.p.z()*w_T_cn.M[1,2]],
                   [ w_T_mn.p.z() - w_T_cn.p.z() - cr_T_mr.p.x()*w_T_cn.M[2,0] - cr_T_mr.p.y()*w_T_cn.M[2,1] - cr_T_mr.p.z()*w_T_cn.M[2,2]]])             
    #print (B)
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
        #marker w.r.t. camera
        
        self._w_T_ee.append(w_T_ee)
        self._c_T_m.append(c_T_m)
    def set_intial_frames(self,w_T_c,ee_T_m):
        #marker w.r.t. camera
        self.w_T_c=w_T_c        # marker in ee
        self.ee_T_m=ee_T_m
        
    def compute_frames(self,r_eq=1):
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
            sc=1
            W=num.matrix(num.diag(num.array([1,1,1,sc*r_eq,sc*r_eq,sc*r_eq,sc,sc,sc])))

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
    from random import random
    import matplotlib.pyplot as plt
    
    def random_pose(angle_range=num.pi/4, pos_range=0.5):
        return PyKDL.Frame(PyKDL.Rotation.RPY(angle_range*random(),angle_range*random(),angle_range*random()),
                PyKDL.Vector(pos_range*random(),pos_range*random(),pos_range*random()))
    
    #define real poses of camera and marker
    w_TR_c=random_pose(num.pi/2,0.5)
    ee_TR_m=random_pose(num.pi/2,0.5)
    #w_TR_c=PyKDL.Frame.Identity();
    #ee_TR_m=PyKDL.Frame.Identity();
    
    crc=camera_robot_calibration()
    #generate some positions of the robot, and make some camera measurements
    for i in range(10):
        #generate robot position
        w_T_ee=random_pose()
        #measure the frame of the marker w.r.t. the camera, plus some noise 
        #(noise on angle is not necessary as only marker position is used)
        c_T_m=w_TR_c.Inverse()*w_T_ee*ee_TR_m*random_pose(0,0.01)
        #store the frames...
        crc.store_frames(w_T_ee,c_T_m)
    
    residue_max=[]
    residue_mod=[]
    #residue_all=None
    #x_bar=None
    #y_bar=None
    n_comp=6
    for i in range(n_comp):
        residue=crc.compute_frames();
        r2=residue.transpose()*residue
        residue_mod.append( num.sqrt (r2[0,0]))
        residue_max.append(num.max(residue))
   #     if residue_all==None:
    #        residue_all=residue.transpose()
     #       x_bar=num.matrix(range(9))
      #      y_bar=num.matrix([i for x in range(9)])
      #  else:
      #      residue_all=num.hstack([residue_all,residue.transpose()])
      #      x_bar=num.hstack([x_bar,num.matrix(range(9))])
      #      y_bar=num.hstack([y_bar,num.matrix([i for x in range(9)])])

            
    
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
    plt.show()
    
            

