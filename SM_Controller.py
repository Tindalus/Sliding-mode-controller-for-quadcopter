import numpy as np


def Controller(State:list, DesiredState:list, Parameters:list, Constants:list,Ts:float,T4:float,T:float, DesiredAccelerations:list = [0,0,0,0,0,0]):
    """
    This function generates control signal in the closed loop system

    Args:
        State (list (12)): Current state of the system.
        DesiredState (list (12)): List of desired trajectory coordinates.
        Parameters (list (12)): List of controller paremeters.
        Constants (list (15)): List of model characteristics: K, m, g, l, b, kftz, kftx, kfty, kfrz, kfrx, kfry, J_x, J_y, J_z, J_r.
        Ts (float): Numerical scheme step.
        T4 (float): Total angular velocity.
        T (float): Total thrust.
        DesiredAccelerations (list (6), optional): list of desired accelerations. Rarely used, default [0,0,0,0,0,0].
    Returns:
        list: list of 6 input signals in order: [u1,u2,u3,tau_psi,tau_phi,tau_theta]
    """

    
    #unloading Constants
    K,m,g,l,b,kftz,kftx,kfty,kfrz,kfrx,kfry,J_x,J_y,J_z,J_r = Constants[:]

    #Forming error wector
    Errors = np.array(DesiredState)-np.array(State)
    
    #Forming sliding space variables
    Svars = np.array([Errors[2*i+1] + Parameters[i]*Errors[2*i] for i in range(int(len(Errors)/2))])

    #Calculating additional parameters
    a1 = (J_y - J_z)/J_x
    b1 = (l)/J_x
    a2 = (J_z - J_x)/J_y
    b2 = (l)/J_y
    a3 = (J_x - J_y)/J_z
    b3 = (l)/J_z

    
    u3 = m*(1/(np.cos(State[6])*np.cos(State[8])))*(DesiredAccelerations[2] + Parameters[0]*Errors[1]*Ts + (kftz/m)*State[1]*Ts + Parameters[6]*(Svars[0])/(np.abs(Svars[0])+0.48))

    u1 = m*(1/T)*(DesiredAccelerations[0] + Parameters[1]*Errors[3]*Ts + (kftx/m)*State[3]*Ts + Parameters[7]*(Svars[1])/(np.abs(Svars[1])+0.3)) 

    u2 = m*(1/T)*(DesiredAccelerations[1] + Parameters[2]*Errors[5]*Ts + (kfty/m)*State[5]*Ts + Parameters[8]*(Svars[2])/(np.abs(Svars[2])+0.3))

    tau_psi = J_z*(DesiredAccelerations[5] + Parameters[3]*Errors[7]*Ts - a3*State[7]*State[9]*(1/Ts)
                    + b3*(kfrz)*(State[11]**2)*Ts + Parameters[9]*(Svars[3])/(np.abs(Svars[3])+0.4))

    tau_phi = J_x*(DesiredAccelerations[3] + Parameters[4]*Errors[9]*Ts - a1*State[9]*State[11]*(1/Ts)
                    + (-T4)*(J_r/J_x)*State[9]*Ts + b1*(kfrx)*(State[7]**2)*Ts + Parameters[10]*(Svars[4])/(np.abs(Svars[4])+0.4))

    tau_theta = J_y*(DesiredAccelerations[4] + Parameters[5]*Errors[11]*Ts - a2*State[7]*State[11]*(1/Ts)
                      + (-T4)*(J_r/J_y)*State[7]*Ts + b2*(kfry)*(State[9]**2)*Ts + Parameters[11]*(Svars[5])/(np.abs(Svars[5])+0.4))
    
    #u3 is respozible for z coord control, u1 for x and u2 for y
    return [u1,u2,u3,tau_psi,tau_phi,tau_theta]
