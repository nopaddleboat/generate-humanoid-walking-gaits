import numpy as np
from numpy.linalg import matrix_power

class MPC(object):
    '''
    x_k+1= A*x_k + B*u_k
    y_k  = C*x_k + D*u_k
    x_k: n x 1
    u_k m x 1
    A: n x n
    B: n x m
    C: p x n
    D: p x m
    x_k is known, u_k unknown, to be solved
    Y = G*x_k+HU where U=[u_k, u_k+1, u_k+Np-1]  #future output predictions
    G: Np*p x n
    H: Np*p x Np*m    
    '''

    def __init__(self,A: np.ndarray, B: np.ndarray,  C: np.ndarray, D=[], Np=30, rho=0.01) -> None:
        n=A.shape[0]
        m=B.shape[1]
        p=C.shape[0]
        self.rho=rho
        self.A=A
        self.B=B
        self.C=C        
        self.Np=Np
        self.G=np.zeros((Np*p,n))
        self.H=np.zeros((Np*p,Np*m))
        self.deltaU=np.zeros((Np,Np*m))
        if len(D) == 0:
            D=np.zeros((p,m))
        self.D=D

        for k in range(Np):
            self.G[k*p:(k+1)*p,:]=C @ matrix_power(A,k)
            if k==0:
                self.H[0:p,0:m]=D
            else:
                self.H[ k*p:(k+1)*p, 0:m ]= C @ matrix_power(A, k-1) @ B
                self.H[ k*p:(k+1)*p,m:]= self.H[ (k-1)*p:k*p, 0:(Np-1)*m ]
            if k != Np-1:
                self.deltaU[k, k*m:(k+1)*m]= np.ones((1,m))
                self.deltaU[k, (k+1)*m:(k+2)*m]= -np.ones((1,m))
            
        R=self.deltaU.transpose() @ self.deltaU*rho
        self.Q=self.H.transpose() @ self.H+R        

    def calculate_desired_input(self, ref:np.ndarray):
        #Y_R: p*L x 1               
        n=self.A.shape[0]
        m=self.B.shape[1]
        p=self.C.shape[0]
        k_loop=int(ref.shape[0]/p)
        Y_R=np.append(ref, np.ones((self.Np*p+1,))*ref[-1])
        traj_u=np.zeros((k_loop,m))
        traj_y=np.zeros((k_loop,m))
        traj_xk=np.zeros((k_loop,n))
        x_k=np.zeros((n,))        
        Np=self.Np
        for k in range(k_loop):
            P= (x_k.transpose() @ self.G.transpose()- Y_R[ k*p:k*p+p*Np].transpose()) @ self.H
            Uopt=- np.linalg.solve(self.Q, P)
            uk=Uopt[0:m]
            traj_u[k,:]=uk.transpose()

            x_k1=self.A @ x_k + self.B @ uk
            yk= self.C @ x_k + self.D @ uk
            x_k=np.copy(x_k1)
            traj_y[k,:]=yk.transpose()
            traj_xk[k,:]=x_k.transpose()

        return traj_u, traj_y, traj_xk









        
    