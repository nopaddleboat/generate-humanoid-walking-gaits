import numpy as np
from factor import *
from ZmpRef import *
import matplotlib.pyplot as plt
from common_paras import *

import MPC as mpc

class CoMRef(object):
    def __init__(self, zmp_traj: ZmpRef, 
                 solver='MPC',
                 flag_plot=False):
        #solver: LCQP or MPC
        self.zmp_traj = zmp_traj
        self.dt=0.005 #sampling time
        self.t_end= self.zmp_traj.footsteps.time[-1]
        self.com_z=0.8
        self.g=9.8        
        self.x_opt=[]
        self.y_opt=[]
        self.N=int(self.t_end/self.dt)
        self.L=self.com_z/self.g
        self.solver=solver
        self.flag_plot=flag_plot        

        if solver == 'MPC':
            self.x_traj,self.y_traj=self.solve_CoM_traj_mpc()        
        elif solver == "LCQP":
            self.x_traj = self.solve_CoM_traj(0) #sove x trajectory
            self.y_traj =self.solve_CoM_traj(1) #y
        else:
            pass

    def solve_CoM_traj_mpc(self):
        L=self.L
        A=np.array([[1,self.dt],[0, 1]])
        B=np.array([[0],[self.dt]])
        C=np.array([[1,0]])
        D=np.array([-L])
        mpc_model=mpc.MPC(A,B,C,D,Np=200, rho=5)
        ref_x=np.zeros((self.N,))
        ref_y=np.zeros((self.N,))
        for k in range(self.N):
            ref_x[k]=self.zmp_traj(self.dt*k)[0]
            ref_y[k]=self.zmp_traj(self.dt*k)[1]
        x_traj_u, x_traj_y, x_traj_xk = mpc_model.calculate_desired_input(ref_x)
        y_traj_u, y_traj_y, y_traj_xk = mpc_model.calculate_desired_input(ref_y)
        if self.flag_plot:
            plt.subplot(1,2,1)
            plt.plot(ref_x)
            plt.plot(x_traj_y,'--')
            plt.subplot(1,2,2)
            plt.plot(ref_y)
            plt.plot(y_traj_y,'--')
        return x_traj_xk[:,0], y_traj_xk[:,0]

    def solve_CoM_traj(self, dir:int):
        '''
        dir: 0 , x direction, 1 y direction
        '''
        # u = ddc  //x=[c dc]
        # y = c - z/g*u      
        # X=[c_k, dc_k, u_k]  
        N = self.N
        nx = 3
        L=self.L        
        f = FactorGraph(nx, N)
        M1=np.array([[0,1,self.dt],[1,self.dt,0]])
        M2=np.array([[0,1,0],[1,0,0]])
        Mf=eye(1)
        Mc=np.array([[1,0,-L]])
        #construct factor graph
        for k in range(N-1):
            f.add_factor_constraint([Factor(k, M1), Factor(k+1, -M2)],zero(2))
        
        for k in range(N):
            traj_x=self.zmp_traj(self.dt*k)[dir]
            f.add_factor([Factor(k,Mc)],Mf*traj_x)
        #solve for CoM trajectories
        # opt=f.solve() #use scipy
        opt=np.array(f.solve_cvx()) #use cvx
        if dir == 0:
            self.x_opt=np.copy(opt)
        else:
            self.y_opt=np.copy(opt)            
        return np.reshape(opt[0::3],len(opt[0::3]))
 
    def __call__(self, t):     
        idx=int(t/self.dt) 
        return np.array([self.x_traj[idx],self.y_traj[idx],z_CoM])    

    def plot_ctrl_error_x(self):
        if self.solver != "LCQP":
            return
        #verify solution of LCQP along x direction
        ref_x= np.zeros((self.N,1))
        act_u= self.x_opt[2::3]
        act_c= self.x_opt[0::3]
        act_dc=self.x_opt[1::3]
        ddc= (act_dc[1:-1]-act_dc[0:-2])/self.dt
        act_x=act_c-self.L*act_u
        for k in range(self.N):
            ref_x[k]=self.zmp_traj(self.dt*k)[0]
        plt.subplot(1,2,1)
        plt.plot(ref_x)
        plt.plot(act_x,'x')
        plt.subplot(1,2,2)
        plt.plot(act_u)
        plt.plot(ddc,'x')
        
        plt.show()
    def plot_ctrl_error_y(self):
        if self.solver != "LCQP":
            return
        ref_y= np.zeros((self.N,1))
        act_u= self.y_opt[2::3]
        act_c= self.y_opt[0::3]
        act_dc=self.y_opt[1::3]
        ddc= (act_dc[1:-1]-act_dc[0:-2])/self.dt
        act_y=act_c-self.L*act_u
        for k in range(self.N):
            ref_y[k]=self.zmp_traj(self.dt*k)[1]
        plt.subplot(1,2,1)
        plt.plot(ref_y)
        plt.plot(act_y,'x')
        plt.subplot(1,2,2)
        plt.plot(act_u)
        plt.plot(ddc,'x')
        
        plt.show()


    def plot_com_traj(self):
        plt.plot(self.x_traj, self.y_traj)
        plt.show()
        



