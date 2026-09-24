import numpy as np
import math

class Potential:
    def __init__(self, xc=0.0, yc=0.0, R=1.0):
        self.xc = xc
        self.yc = yc   
        self.R = R


    def s(self, x, y):
        return ((x - self.xc)**2 + (y - self.yc)**2) / self.R**2 - 1.0

    def phi(self, x, y):
        s_val = self.s(x, y)
        return s_val * s_val

    def grad_phi(self, x, y):
        s_val = self.s(x, y)
        
        grad_s = np.array([2.0 * (x - self.xc) / self.R**2,
                           2.0 * (y - self.yc) / self.R**2])
        # grad phi = 2 s grad(s)
        return 2.0 * s_val * grad_s

    def force(self, x, y, beta=1, k = 20):
    #def force(self, x, y, beta=3, k = 50):
        g = self.grad_phi(x, y)      
        norm_g = math.sqrt(g[0]**2 + g[1]**2)
        #norm_g = 1 
        if norm_g < 1e-9:      
            return np.zeros(2)
        
        return -k*g / (norm_g ** beta)  
    