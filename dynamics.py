import numpy as np

def dynamics(Ts:float, X:list, U:list, RealityCheck:bool):
        """
        This funtion is used to calculate one step of quadcopter dynamics.
        
        Args:
                Ts (float): Timestep of your grid.
                X (listlike): State vector of the previous step. Must be (12,1). See paper for stucture.
                U (listlike): Input vector applied to the current step. Must be (6,1). See paper for stucture.
                RealityCheck (bool): bool for cheking if the function used in simulation or control law synthesis.
        
        Returns:
                X1 (listlike): State vector (12,1) of the current step.\n
                T (float): Total angular velocity the rotation speed of all four propellers. Might be used to calculate control signal.\n
                T4 (float): Total angular velocity.\n
        """



        #check for real or predictive realization of this function
        #essentially setup for robustness test
        if RealityCheck == 1: 
                m = 0.65*1.25
        elif RealityCheck == 0:
                m = 0.65
        

        ###Parameters###
        K = 0.00000298 #amplification coefficient
        # m = 0.65 #mass
        g = 9.81
        l = 0.23 #distance from mass centre to rotor
        b = 0.0000313
        #coefficients of aerodynamic drag to movement
        kftz = 0.001
        kftx = 0.00075
        kfty = 0.00075
        #coefficients of aerodynamic drag to rotation
        kfrz = 0.001
        kfrx = 0.00075
        kfry = 0.00075
        #moments of inertia of the propellers relative to the respective axes
        J_x = 0.0075
        J_y = 0.0075
        J_z = 0.013
        J_r = 0.00005
        
        
        x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12 = X[0][0], X[1][0], X[2][0], X[3][0], X[4][0], X[5][0], X[6][0], X[7][0], X[8][0], X[9][0], X[10][0], X[11][0]
        u1, u2, u3, tau_phi, tau_theta, tau_psi = U[0][0], U[1][0], U[2][0], U[3][0], U[4][0], U[5][0]

        #Torques and angular velocities simulation
        a1 = (J_y - J_z)/J_x
        a2 = (J_z - J_x)/J_y
        a3 = (J_x - J_y)/J_z
        b1 = (l)/J_x
        b2 = (l)/J_y
        b3 = (l)/J_z
        T1 = np.sqrt((u3 + m*g)**2 + u1**2 + u2**2)
        o1 = T1/(4*K) - J_y*tau_theta/(2*K*l) - J_z*tau_psi/(4*b)
        o2 = T1/(4*K) - J_x*tau_phi/(2*K*l) + J_z*tau_psi/(4*b)
        o3 = T1/(4*K) + J_y*tau_theta/(2*K*l) - J_z*tau_psi/(4*b)
        o4 = T1/(4*K) + J_x*tau_phi/(2*K*l) + J_z*tau_psi/(4*b)
        T = K*(o1+o2+o3+o4)
        T2 = l*K*( o4 - o2)
        T3 = l*K*( o3 - o1)
        T4 = b*( o2 + o4 - o3 - o1)
 
        #disturbances
        D = [0,0,0,0,0,0]

        #new step
        x21 = x2 + T*Ts*(1/m)*(np.cos(x7)*np.cos(x9)) - g*Ts - Ts*(kftz/m)*x2 + D[0]     #z_dot
        x41 = x4 + T*Ts*(1/m)*(np.cos(x7)*np.sin(x9)*np.cos(x11) + np.sin(x7)*np.sin(x11)) - Ts*(kftx/m)*x4 + D[1]     #x_dot
        x61 = x6 + T*Ts*(1/m)*(np.cos(x7)*np.sin(x9)*np.sin(x11) - np.sin(x7)*np.cos(x11)) - Ts*(kfty/m)*x6 + D[2]     #y_dot
        x81 = x8 + a1*(1/Ts)*x10*x12 + b1*Ts*T2 - (-T4)*Ts*(J_r/J_x)*x10 - b1*Ts*(kfrx)*(x8**2) + D[3]       #phi_dot
        x101 = x10 + a2*(1/Ts)*x8*x12 + b2*Ts*T3 - (-T4)*Ts*(J_r/J_y)*x8 - b2*Ts*(kfry)*(x10**2) + D[4]      #theta_dot
        x121 = x12 + a3*(1/Ts)*x8*x10 + b3*Ts*T4 - b3*Ts*(kfrz)*(x12**2) + D[5]        #psi_dot

        #updating states for the next step
        x2 = x21
        x4 = x41
        x6 = x61
        x8 = x81
        x10 = x101
        x12 = x121

        #integrating
        x1_ = x1 + Ts*x2
        x3_ = x3 + Ts*x4
        x5_ = x5 + Ts*x6
        x7_ = x7 + Ts*x8
        x9_ = x9 + Ts*x10
        x11_ = x11 + Ts*x12

        # x1 = z
        # x3 = x
        # x5 = y
        # x7 = phi
        # x9 = theta
        # x11 = psi

        X1 = np.zeros(shape=(12,1))
        X1[:,0] = x1_, x2, x3_, x4, x5_, x6, x7_, x8, x9_, x10, x11_, x12
        return X1,T,T4
