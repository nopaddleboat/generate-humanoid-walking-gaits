import numpy as np
from foot_steps import *
import matplotlib.pyplot as plt

class ZmpRef(object):
    def __init__(self, footsteps):
        self.footsteps = footsteps
 
    def __call__(self, t):        
        left=np.array(self.footsteps.get_left_position(t))
        left_next=np.array(self.footsteps.get_left_next_position(t))
        right=np.array(self.footsteps.get_right_position(t))
        right_next=np.array(self.footsteps.get_right_next_position(t))
        t_start=self.footsteps.get_phase_start(t)
        t_total=self.footsteps.get_phase_duration(t)
        k=(t-t_start)/t_total
        act_left= left+(left_next-left)*k
        act_right= right+(right_next-right)*k

        return np.array((act_left+act_right)/2)

def plot_zmp(footsteps:FootSteps):
    test=ZmpRef(footsteps)
    t_max=footsteps.time[-1]
    dt=0.1
    N=int(t_max/dt+1)
    zmp_traj=np.zeros((N,2))
    left_traj=np.zeros((N,2))
    right_traj=np.zeros((N,2))
    for k in range(N):
        t_q=dt*k
        zmp_traj[k,:]=np.copy(test(t_q))
        left_traj[k,:]=footsteps.get_act_left_position(t_q)
        right_traj[k,:]=footsteps.get_act_right_position(t_q)
    plt.plot(zmp_traj[:,0],zmp_traj[:,1],'o')
    plt.plot(left_traj[:,0],left_traj[:,1],'x')
    plt.plot(right_traj[:,0],right_traj[:,1],'x')
    plt.show()
    