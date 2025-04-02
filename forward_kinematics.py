import numpy as np
import matplotlib.pyplot as py
import numpy.linalg as linalg


# b and p are 6 long vectors of xyz coordinates of joint attachments
# L is the length of the legs
# Return (x,y,z,alpha,beta,gamma)


def fk(b_position, p_position,L):

    number_of_iterations = 0
    max_interations = 100
    tolerance_a = 1e-3
    # Initial Guess (x,y,z,alpha,beta,gamma) in deg
    a = [0,0,10,0,0,0]
    a = np.array(a).transpose()
    
    # TO RAD
    alpha = np.deg2rad(a[3])
    beta = np.deg2rad(a[4])
    gamma = np.deg2rad(a[5])
    
    
    while max_interations > number_of_iterations:
        number_of_iterations += 1
        
        # Calculate COSINE
        c_alpha = np.cos(alpha)
        c_beta = np.cos(beta)
        c_gamma = np.cos(gamma)
    
        # Calculate SINE
        s_alpha = np.sin(alpha)
        s_beta = np.sin(beta)
        s_gamma = np.sin(gamma)
        
        # Calculate R
        Rxyz = np.array(
            [[c_gamma*c_beta, c_gamma*s_beta*s_alpha - s_gamma*c_alpha, c_gamma*s_beta*c_alpha + s_gamma*s_alpha] 
            ,[s_gamma*c_beta, s_gamma*s_beta*s_alpha + c_gamma*c_alpha, s_gamma*s_beta*c_alpha - c_gamma*s_alpha] 
            ,[-s_beta, c_beta*s_alpha, c_beta*c_alpha]])
        
        
        x_bar = a[0:3] - b_position
        
        # Platform Rotation with respect to base
        x_uvw = np.zeros(p_position.shape)
        for i in range(6):
            x_uvw[i,:] = np.dot(Rxyz,p_position[i,:])
        
        l_i = np.sum(np.square(x_bar + x_uvw), 1)
        
        f = -1 * (l_i-np.square(L))
        sum_F = np.sum(np.abs(f))
        
        # Checking if sum is within tolerance i.e. if we have found the lengths
        if sum_F < tolerance_a:
            break
        
        df_da = np.zeros((6,6))
        df_da[:, 0:3] = 2*(x_bar + x_uvw)
        
        for i in range(6):
            #Numpy * is elementwise multiplication!!
            #Indicing starts at 0!
            #dfda4 is swapped with dfda6 for magic reasons!  
            df_da[i, 5] = 2*(-x_bar[i,0]*x_uvw[i,1] + x_bar[i,1]*x_uvw[i,0]) #dfda4
            df_da[i, 4] = 2*((-x_bar[i,0]*c_gamma + x_bar[i,1]*s_gamma)*x_uvw[i,2] \
                            -(p_position[i,0]*c_beta + p_position[i,1]*s_beta*s_alpha)*x_bar[i,2]) #dfda5
            df_da[i, 3] = 2*p_position[i, 1]*(np.dot(x_bar[i,:],Rxyz[:,2])) #df_da
            
        delta_a = linalg.solve(df_da, f)
        
        if np.abs(np.sum(delta_a)) < tolerance_a:
            break
        else:
            a = a + delta_a
            
    return a
            
        
        
        
        
    
    
    
    
