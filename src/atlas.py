# NOTE: this example needs gepetto-gui to be installed
# usage: launch gepetto-gui and then run this test
 
import pinocchio as pin
import numpy as np
import sys
import os
from os.path import dirname, join, abspath
 
from pinocchio.visualize import GepettoVisualizer
import time

from pinocchio.robot_wrapper import RobotWrapper

from foot_steps import *
from common_paras import *

from scipy.optimize import fmin_bfgs
 
class Atlas(object):
    # Load the URDF model.
    # Conversion with str seems to be necessary when executing this file with ipython
    def __init__(self):
       model_dir = join(dirname(dirname(str(abspath(__file__)))),"model")
       
       model_path = model_dir
       mesh_dir = model_dir
       urdf_filename = "atlas_v4_with_multisense_STL.urdf"
       urdf_model_path = join(model_path,urdf_filename)

       self.robot = RobotWrapper.BuildFromURDF(urdf_model_path,mesh_dir)
       self.robot.initViewer()           

       self.l_foot_rgb = [0.0, 1.0, 0.2, 1.0]      
       self.r_foot_rgb = [0.0, 0.2, 1.0, 1.0]  
       self.scale=1
       self.foot_size=[0.3*self.scale,0.1*self.scale,0.001*self.scale]
       self.feet_indx= [self.robot.index('l_leg_akx'), self.robot.index('r_leg_akx')]
       self.torso_idx = self.robot.index('back_bkx')

       self.init_view() 

    # Initialize the viewer.
    def init_view(self):        
       try:
          self.robot.loadViewerModel("atlas")
       except AttributeError as err:
          print("Error while loading the viewer model. It seems you should start gepetto-viewer")
          print(err)
          sys.exit(0)
       self.q0=(self.robot.q0)
      #  self.q0[21]=0.5
      #  self.q0[20]=0.5
       self.q_pel=np.zeros(7)
       self.q_pel[6]=1
       self.robot.framesForwardKinematics(self.q0)

       T_b_feet=self.transform_base(0) #left foot
       T_feet_b=T_b_feet.inverse()

       #move to left foot position
       self.q_pel[0:3]=T_feet_b.translation
       self.q_pel[0:2]=self.q_pel[0:2]+left_foot_coord
       self.q_pel[2]=self.q_pel[2]+d_foot_ankle
       q_w_b=pin.Quaternion(T_feet_b.rotation)
       self.q_pel[3:7]=[q_w_b.x, q_w_b.y, q_w_b.z, q_w_b.w]

      #  self.robot.display(self.q0)

    def plot_foot_steps(self, planned_feet: FootSteps):
       n_left=0
       n_right=0
       for t in planned_feet.time[0:-2]:
          pos_l_foot=[0,0,d_foot_ankle*self.scale,1,0,0,0]
          pos_r_foot=[0,0,d_foot_ankle*self.scale,1,0,0,0]
          feet_type=planned_feet.get_phase_type(t)
          if feet_type == 'right' or feet_type == 'none':            
             n_left=n_left+1 
             tmp_name="world/lfoot"+str(n_left)
             self.robot.viewer.gui.addBox(tmp_name, self.foot_size[0],self.foot_size[1],self.foot_size[2], self.l_foot_rgb)  
             pos_l_foot[0:2]=planned_feet.get_left_position(t)             
             pos_l_foot[0]=pos_l_foot[0]*self.scale
             pos_l_foot[1]=pos_l_foot[1]*self.scale
             self.robot.viewer.gui.applyConfiguration(tmp_name, pos_l_foot)
          else: # feet_type == 'left': 
             n_right=n_right+1 
             tmp_name="world/rfoot"+str(n_right)
             self.robot.viewer.gui.addBox(tmp_name, self.foot_size[0],self.foot_size[1],self.foot_size[2], self.r_foot_rgb)  
             pos_r_foot[0:2]=planned_feet.get_right_position(t) 
             pos_r_foot[0]=pos_r_foot[0]*self.scale
             pos_r_foot[1]=pos_r_foot[1]*self.scale            
             self.robot.viewer.gui.applyConfiguration(tmp_name, pos_r_foot)
    def render_scene(self):       
      #  self.robot.display(q0)
      self.render_robot()
      self.robot.viewer.gui.refresh()  # Refresh the window.
    
    def transform_base(self, supporting_leg: int):
       #supporting_leg: 0 left, 1 right       
       return self.robot.data.oMi[self.feet_indx[supporting_leg]]  
    
    def render_robot(self):
       self.robot.viewer.gui.applyConfiguration("world/atlas", self.q_pel.tolist())  
       self.robot.display(self.q0)
    
    def render_CoM(self, q_pel):
       rgbt = [1.0, 0.2, 0.0, 1.0]  
       tmp_name="world/CoM"
       CoM=self.robot.com(self.q0)
       self.robot.viewer.gui.addSphere(tmp_name, .1, rgbt)  # .1 is the radius
      #  quat=pin.Quaternion(np.array(self.q_pel[3:7]))       
      #  se3_b_feet=pin.SE3(quat,np.array(self.q_pel[0:3]))
       se3_b_feet=pin.XYZQUATToSE3(q_pel)
       CoM_act=se3_b_feet.act(CoM)
       
       pos_CoM=[0,0,0,1,0,0,0]       
       pos_CoM[0:3]=CoM_act
       self.robot.viewer.gui.applyConfiguration(tmp_name, pos_CoM)

    def render_feet(self, q_pel):
       feet=["world/robot_lf","world/robot_rf"]
       rgbt = [1.0, 0.7, 0.0, 1.0]  
       lf=self.robot.data.oMi[self.feet_indx[0]].translation #left
       rf=self.robot.data.oMi[self.feet_indx[1]].translation #right
       se3_b_feet=pin.XYZQUATToSE3(q_pel)
       lf_w=se3_b_feet.act(lf)
       rf_w=se3_b_feet.act(rf)
       pos_lf=[0,0,0.2,1,0,0,0]  
       pos_rf=[0,0,0.3,1,0,0,0] 
       pos_lf[0:3]=lf_w
       pos_rf[0:3]=rf_w
       self.robot.viewer.gui.addSphere(feet[0], .06, rgbt)  # .1 is the radius
       self.robot.viewer.gui.addSphere(feet[1], .06, rgbt)  # .1 is the radius       
       
       self.robot.viewer.gui.applyConfiguration(feet[0], pos_lf)
       self.robot.viewer.gui.applyConfiguration(feet[1], pos_rf)   
      
    def IK_q0_from_feet_CoM(self, 
                            supportingLeg:int,
                            swingLeg: int, 
                            swingAnkle:np.ndarray, 
                            supportAnkle:np.ndarray,
                            CoM:np.ndarray,
                            q:np.ndarray):
              
       def IK_cost(x: np.ndarray):
          q_pel=np.zeros(7)
          self.robot.framesForwardKinematics(x)          
          T_b_feet=self.robot.data.oMi[self.feet_indx[supportingLeg]]  
          T_pelvis_tor=self.robot.data.oMi[self.torso_idx]  
          T_feet_b=T_b_feet.inverse()

          q_pel[0:3]=T_feet_b.translation          
          q_pel[0:2]=q_pel[0:2]+supportAnkle[0:2]#+left_foot_coord
          q_pel[2]=q_pel[2]+d_foot_ankle
          q_w_b=pin.Quaternion(T_feet_b.rotation)
          q_pel[3:7]=[q_w_b.x, q_w_b.y, q_w_b.z, q_w_b.w]
          se3_b_feet=pin.XYZQUATToSE3(q_pel)
          CoM_k=self.robot.com(x)
          f_swing=self.robot.data.oMi[self.feet_indx[swingLeg]].translation 
          T_swing=se3_b_feet.act(self.robot.data.oMi[self.feet_indx[swingLeg]])
          error_pelvis=T_feet_b.rotation[0:3,2]-np.array([0,0,1])
          error_torso=T_pelvis_tor.rotation[0:3,2]-np.array([0,0,1])
          pose1=pin.SE3ToXYZQUAT(T_swing)

          f_swing_w=se3_b_feet.act(f_swing)
          CoM_k_w=se3_b_feet.act(CoM_k)
          
          #constraints:1) swing foot match planned trajectory
          #2) CoM match planned 3) orientation of the support foot stays fixed
          #4) torso stay upward 5) pelvis stay upward 
          #6)try to be as close to the initial joint configuration as possible
          # you can add extra constraints to regulate the arm poses.
          return np.linalg.norm(f_swing_w-swingAnkle)+np.linalg.norm(CoM_k_w-CoM) \
                 +np.abs(pose1[6]-1)+0.2*np.linalg.norm(error_torso) \
                 +0.2*np.linalg.norm(error_pelvis)+0.02*np.linalg.norm(x-q)
       
       class CallbackLogger:
          def __init__(self):
             self.nfeval = 1
          def __call__(self,x):
             print('===CBK=== {0:4d} {1: 3.6f} {2: 3.6f} {3: 3.6f}'.format(self.nfeval, x[0], x[1], IK_cost(x)))
             self.nfeval += 1

       #Given CoM and foot coordinates, use IK to solve joint angles
       #use BFGS
       x0=np.copy(q)
       opt_q= fmin_bfgs(IK_cost, x0, callback=CallbackLogger(),maxiter=30) #disp=False
       print('*** Xopt in BFGS =', opt_q)

       return opt_q
    
    def render_robot_walking(self, 
                             q0, 
                             supportingLeg, 
                             swingLeg,
                             supportAnkle):
       self.q0=np.copy(q0)
       self.robot.framesForwardKinematics(q0)          
       T_b_feet=self.robot.data.oMi[self.feet_indx[supportingLeg]]  
       T_feet_b=T_b_feet.inverse()

       q_pel=np.zeros(7) #pelvis frame
       #move to left foot position
       q_pel[0:3]=T_feet_b.translation
       q_pel[0:2]=q_pel[0:2]+supportAnkle[0:2]#+left_foot_coord
       q_pel[2]=q_pel[2]+d_foot_ankle
       q_w_b=pin.Quaternion(T_feet_b.rotation)
       q_pel[3:7]=[q_w_b.x, q_w_b.y, q_w_b.z, q_w_b.w]

       self.render_CoM(q_pel)       
       self.render_feet(q_pel)
       self.robot.viewer.gui.applyConfiguration("world/atlas", q_pel.tolist())  
       self.robot.display(q0)

       self.robot.viewer.gui.refresh()  # Refresh the window.
       
   
       
       
      
   

       
          
       
       
       
    
       
             
       
       
    
 
# Display a robot configuration.
# rgbt = [1.0, 0.2, 0.2, 1.0]  # red, green, blue, transparency
# robot.viewer.gui.addSphere("world/sphere", .1, rgbt)  # .1 is the radius
# rgbt_blue = [0.0, 0.2, 1.0, 1.0]  
# robot.viewer.gui.addBox("world/lfoot", .3,0.2,0.02, rgbt_blue)  # .1 is the radius
# robot.viewer.gui.addBox("world/rfoot", .3,0.2,0.02, [0.0, 1.0, 0.1, 1.0] )  # .1 is the radius

# #get foot position and use a box to update it.
# l_foot_idx = robot.index('l_leg_akx')
# r_foot_idx = robot.index('r_leg_akx')
# # placement = robot.placement(q0, idx)

# # a_atlas=[0,0,2,1,0,0,0]

# pos_l_foot=[0,0,0,1,0,0,0]
# pos_r_foot=[0,0,0,1,0,0,0]

# a_atlas=[0,0,2,0,0,0,1]
# for k in range(10):
#     robot.framesForwardKinematics(q0)
#     act_Mi_l_foot=robot.data.oMi[l_foot_idx]    
#     act_Mi_r_foot=robot.data.oMi[r_foot_idx]  
#     a_atlas[1]=a_atlas[1]+0.2
#     pos_l_foot[0:3]=act_Mi_l_foot.translation+a_atlas[0:3]
#     pos_r_foot[0:3]=act_Mi_r_foot.translation+a_atlas[0:3]
#     robot.viewer.gui.applyConfiguration("world/sphere", (.5, .1, 2.2, 1.,0.,0.,0. ))    
#     robot.viewer.gui.applyConfiguration("world/lfoot", pos_l_foot)  
#     robot.viewer.gui.applyConfiguration("world/rfoot", pos_r_foot)  
#     robot.viewer.gui.applyConfiguration("world/pinocchio", a_atlas)      
#     robot.display(q0)
#     robot.viewer.gui.refresh()  # Refresh the window.
#     time.sleep(1)
    