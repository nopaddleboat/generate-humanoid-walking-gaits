import MPC as mpc
import numpy as np
import matplotlib.pyplot as plt

def TEST_MPC():

    Fs=100
    dt=1/Fs
    N=5000
    t=np.arange(0,N)*dt

    A=np.array([[np.cos(dt), np.sin(dt)],[-np.sin(dt), np.cos(dt)]])
    B=np.array([[1-np.cos(dt), 1-np.cos(dt)],[np.sin(dt), np.sin(dt)]])
    C=np.array([[-1,0],[0,0.87]])
    D=np.array([[1,0],[0,0.2]])

    Y_R=np.array([np.sin(t),np.sin(2*t)]).transpose()
    Y_R=np.reshape(Y_R, C.shape[0]*len(t))

    model=mpc.MPC(A,B,C,D)
    traj_u,traj_y,=model.calculate_desired_input(Y_R)

    for k in range(C.shape[0]):
        plt.subplot(1,2,k+1)
        plt.plot(Y_R[k::2])
        plt.plot(traj_y[:,k],'--')
    plt.show()



