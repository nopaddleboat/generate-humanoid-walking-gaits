from foot_steps import *
from atlas import *
from ZmpRef import *
from factor import *
from CoMRef import *
from AnkleRef import *
from common_paras import *

import pinocchio as pin

from timeit import default_timer as timer

# step 1 foot steps planning
# Define 6 steps forward, starting with the left foot and stoping at the same forward position. 
footsteps = FootSteps([.0, -.18],left_foot_coord,) #right, left
footsteps.add_phase(.3, 'none')
footsteps.add_phase(.7, 'left', [.3, -0.18]) #supportingLeg: left; swingLeg: right
footsteps.add_phase(.1, 'none')
footsteps.add_phase(.7, 'right', [.6, 0.1])
footsteps.add_phase(.1, 'none')
footsteps.add_phase(.7, 'left', [.9, -0.18])
footsteps.add_phase(.1, 'none')
footsteps.add_phase(.7, 'right', [1.2, 0.1])
footsteps.add_phase(.1, 'none')
footsteps.add_phase(.7, 'left', [1.5, -0.18])
footsteps.add_phase(.1, 'none')
footsteps.add_phase(.7, 'right', [1.5, 0.1])
footsteps.add_phase(.5, 'none')

#step 2: ZMP reference calculations
# plot_ankle_position(footsteps)
# plot_zmp(footsteps)

zmp_traj=ZmpRef(footsteps)
#step 3: CoM trajectory calculations
com_traj= CoMRef(zmp_traj) #use MPC to solve the CoM trajectory
# com_traj= CoMRef(zmp_traj, solver="LCQP") #use LCQP instead
# com_traj.plot_com_traj() #plot the CoM trajectory

#step 4: ankle pose calculations
left_ank=LeftAnkleRef(footsteps)
right_ank=RightAnkleRef(footsteps)
robot_atlas=Atlas()
robot_atlas.render_scene()
#plot planned foot steps
robot_atlas.plot_foot_steps(footsteps)

t=0
dt=0.04 #sampling time
ankSupport=np.array(left_ank(0))
robot_atlas.render_robot_walking(supportingLeg=0, 
                                 swingLeg=1, 
                                 q0=robot_atlas.q0,
                                 supportAnkle=ankSupport)   
q_ik=np.copy(robot_atlas.q0)
q_opt=np.copy(q_ik)
#starting with left supporting leg
supportingLeg=0 #left
swingLeg=1  
#step 5: use IK to solve robot pose in joint space and perform simulation
while t<footsteps.time[-1]:
    
    if footsteps.get_phase_type(t) == 'right':
        supportingLeg=1
        swingLeg=0
        ankSwing=np.array(left_ank(t))
        ankSupport=np.array(right_ank(t))
        print('t:',t,' swing left:', ankSwing, ' support ', ankSupport)
    elif footsteps.get_phase_type(t) == 'left':   
        supportingLeg=0 #left
        swingLeg=1         
        ankSwing=np.array(right_ank(t))
        ankSupport=np.array(left_ank(t))
        print('t:',t,' swing right:', ankSwing, ' support ', ankSupport)
    else:
        print('Double support phase!')
    
    if footsteps.get_phase_type(t) != 'none':
        q_opt=robot_atlas.IK_q0_from_feet_CoM(supportingLeg=supportingLeg, 
                                    swingLeg=swingLeg,
                                    swingAnkle=ankSwing, 
                                    supportAnkle=ankSupport,
                                    CoM= com_traj(t),
                                    q=q_ik)
    q_ik=np.copy(q_opt)
    robot_atlas.render_robot_walking(supportingLeg=supportingLeg, 
                                     swingLeg=swingLeg, 
                                     q0=q_ik,
                                     supportAnkle=ankSupport)   
     
    
    t=t+dt
    


print('The end!')

