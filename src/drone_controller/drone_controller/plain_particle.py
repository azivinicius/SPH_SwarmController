import math
import numpy as np

def W(q1,q2):
    
    h = q1.h
    k =  q1.distance_to(q2)/h

    if 0<=k<=1: W = 1 - 3/2*k**2 +3/4*k**3
    elif 1<=k<=2: W = 1/4*(2-k)**3
    else: W=0.0

    return 10/(7*math.pi*h**2) * W

def grad_W(q1,q2):
    h = q1.h

    dx = q1.x - q2.x   
    dy = q1.y -q2.y     
        
    if dx==0 and dy==0:
        return 0,0

    k = q1.distance_to(q2)/h
    dkdx = 1/(h*2*math.sqrt(dx**2 +dy**2)) * 2*dx
    dkdy = 1/(h*2*math.sqrt(dx**2 +dy**2)) * 2*dy

    if 0<=k<=1:
        dWdk = -3*k + 9/4*k**2
    elif 1<=k<=2:
        dWdk = -3/4*(2-k)**2
    else: dWdk = 0.0

    dWdk = (10/(7*math.pi*h**2))* dWdk

    dWdx = dWdk*dkdx
    dWdy = dWdk*dkdy
    
    return dWdx,dWdy

def viscosity(qi,qj):
    zetta = [1.0, 2.0]
    # zetta = [0.6419, -0.2906]
    qij = [qj.x - qi.x, qj.y - qi.y]
    vij = [qj.vel[0] - qi.vel[0], qj.vel[1] - qi.vel[1]]
    pij = (qi.rho + qj.rho)/2
    cij = (qi.c + qj.c)/2
    eta_2 = 0.01*qi.h**2
    
    vij_qij = np.dot(vij,qij)

    uij = (qi.h*vij_qij)/(qi.distance_to(qj)**2 + eta_2)
    
    # R = 1.5
    # e = 0.3
    # uij = (qi.h*vij_qij)/((qi.distance_to(qj) - (2*R +e))**2)

    if vij_qij > 0: return 0.0
    else:
        visc = 1/pij*(-zetta[0]*cij*uij + zetta[1]*uij**2)
        return visc


class Particle:
    def __init__(self,id,x,y,m):

        self.id = id

        self.x = x
        self.y = y

        self.h = 1.0

        self.m = m
        self.rho = 1000.0
        self.ext_force = [0.0, 0.0]
        
        self.e = 1.0
        self.B = (200 * self.rho * 9.8 * (1/98)) / 1
        self.P = self.B*((self.rho/1000)**1 -1)

        self.c =  math.sqrt(1*(self.P+self.B)/self.rho)
        self.e = 1.0

        self.vel = [0.0,  0.0]

    def distance_to(self, q):
        return math.sqrt((q.x - self.x)**2 + (q.y-self.y)**2)
    
    def pos_norm(self):
        return math.sqrt((self.x)**2 + (self.y)**2)
    
    def vel_norm(self):
        return math.sqrt((self.vel[0])**2 + (self.vel[1])**2)
    
    def next_rho(self, q_list):
        pi = 0.0
        
        for q in q_list:
            # if q.id == self.id:
            #     continue
            pi += q.m * W(self, q)
        #self.rho = pi
        return pi


    def next_pressure(self):
        if self.rho < 1e-6:
            self.P = 0.0
            self.c = 0.0
            return 0.0
        self.B = 20.0 * self.rho          
        P = self.B * ((self.rho / 1000.0) - 1.0)   
        self.c = math.sqrt((P + self.B) / self.rho)  
        #self.P = P

        return P

    def next_vel(self,q_list):
        dvdt = [0.0, 0.0]
        for q in q_list:
            if q.id == self.id:
                continue
            dWdx,dWdy = grad_W(self, q)

            if self.rho < 1e-6 or q.rho < 1e-6:                
                P_term = 0.0
            else: P_term = (self.P/self.rho**2)+(q.P/q.rho**2)+ viscosity(self,q)
        
            dvdt[0] += q.m*P_term*dWdx
            dvdt[1] += q.m*P_term*dWdy
    
        zetta = 5
        
        dvdt[0] = -dvdt[0] - zetta*self.vel[0] + self.ext_force[0]
        dvdt[1] = -dvdt[1] - zetta*self.vel[1] + self.ext_force[1]
        

        #self.vel = self.vel + dvdt
        
        return dvdt


    def next_energy(self,q_list):
        e = 0.0
        for q in q_list:

            if q.id == self.id:
                continue
            dW = grad_W(self, q)
            
            vij = [q.vel[0] - self.vel[0], q.vel[1] - self.vel[1]]

            if self.rho < 1e-6 or q.rho < 1e-6:
                P_term = 0.0
            else: P_term = (self.P/self.rho**2)+(q.P/q.rho**2)+ viscosity(self,q)

            e += q.m*P_term*np.dot(vij,dW)
        
        e = 1/2*e
        
        #self.e = e
        
        return e
    def next_state(self, q_list):
        dvdt = [0.0, 0.0]
        e = 0.0
        pi = 0.0

        for q in q_list:
            # Density
            if q.id == self.id:
                continue
            pi += q.m * W(self, q)
            # Pressure
            if self.rho < 1e-6:
                P = 0.0
                self.c = 0.0
            
            self.B = 20.0 * self.rho          
            P = self.B * ((self.rho / 1000.0) - 1.0)   
            self.c = math.sqrt((P + self.B) / self.rho)  

            #Acceleration
            dWdx,dWdy = grad_W(self, q)

            if self.rho < 1e-6 or q.rho < 1e-6:                
                P_term = 0.0
            else: P_term = (self.P/self.rho**2)+(q.P/q.rho**2)+ viscosity(self,q)
        
            dvdt[0] += q.m*P_term*dWdx
            dvdt[1] += q.m*P_term*dWdy
            dW = grad_W(self, q)
            
            vij = [q.vel[0] - self.vel[0], q.vel[1] - self.vel[1]]
            e += q.m*P_term*np.dot(vij,dW)
        
        e = 1/2*e
    
        zetta = 5
        
        dvdt[0] = -dvdt[0] - zetta*self.vel[0] + self.ext_force[0]
        dvdt[1] = -dvdt[1] - zetta*self.vel[1] + self.ext_force[1]
        

        #self.vel = self.vel + dvdt
        
        return pi, P, dvdt, e

    def update_particle(self,rho, P, dvdt, e, dt):
        self.rho = rho
        self.P = P
        self.vel[0] += dvdt[0] * dt
        self.vel[1] += dvdt[1] * dt
        self.e = e
        self.x +=  self.vel[0]*dt
        self.y += self.vel[1]*dt
        