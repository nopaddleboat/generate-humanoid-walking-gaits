
from foot_steps import *
import numpy as np
import matplotlib.pyplot as plt
from common_paras import *

class LeftAnkleRef(object):

    def __init__(self, footsteps: FootSteps) -> None:
        self.footsteps=footsteps

    def __call__(self, t):        
        left=np.array(self.footsteps.get_left_position(t))
        left_next=np.array(self.footsteps.get_left_next_position(t))        
        t_start=self.footsteps.get_phase_start(t)
        t_total=self.footsteps.get_phase_duration(t)
        k=(t-t_start)/t_total

        z_foot=h_lift*np.sin(k*np.pi)+d_foot_ankle
        act_left= left+(left_next-left)*k
        return np.array([act_left[0],act_left[1],z_foot])



class RightAnkleRef(object):

    def __init__(self, footsteps: FootSteps) -> None:        
        self.footsteps=footsteps

    def __call__(self, t):        
        right=np.array(self.footsteps.get_right_position(t))
        right_next=np.array(self.footsteps.get_right_next_position(t))
        t_start=self.footsteps.get_phase_start(t)
        t_total=self.footsteps.get_phase_duration(t)
        k=(t-t_start)/t_total

        z_foot=h_lift*np.sin(k*np.pi)+d_foot_ankle
        act_right= right+(right_next-right)*k
        return np.array([act_right[0],act_right[1],z_foot])

def plot_ankle_position(footsteps:FootSteps):
    tmp_left=LeftAnkleRef(footsteps)
    tmp_right=RightAnkleRef(footsteps)

    t_max=footsteps.time[-1]
    dt=0.01
    N=int(t_max/dt+1)
    zmp_traj=np.zeros((N,2))
    ankle_left_traj=np.zeros((N,3))
    ankle_right_traj=np.zeros((N,3))
    for k in range(N):
        t_q=dt*k
        ankle_left_traj[k,:]=tmp_left(t_q)
        ankle_right_traj[k,:]=tmp_right(t_q)
    ax = plt.figure().add_subplot(projection='3d')
    ax.plot(ankle_left_traj[:,0],ankle_left_traj[:,1],ankle_left_traj[:,2])
    plt.show()
    


        
    