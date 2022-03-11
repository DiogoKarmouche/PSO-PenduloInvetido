from scipy.integrate import solve_ivp
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import quad
import scipy.linalg
from sympy import sin, cos, Matrix,symbols,N,evalf
from sympy.solvers import solve
import pickle
class Sistema:
    def __init__(self):
        # Parâmetros do sistema
        self.g=9.81;                # gravidade [m/s²]
        self.Beq=5.4;               # coeficiente de atrito equivalente
        self.Bp=0.0024;             # coeficiente de atrito do pêndulo
        self.kt=0.00767;            # constante do motor
        self.km=0.00767;            # constante de força contra eletromotriz
        self.Ra=2.6;                # resistencia
        self.Jm=3.9E-7;             # 
        self.rp=6.35E-3;            # raio do pinhão
        self.kg=3.71                # constande de redução realizada pela caixa de transmissão
        self.Eff_m=1;               # eficiencia de transmissão de movimento
        self.Eff_g=1;               # eficiencia de transmissão de movimento
        self.Mc=0.57+0.37;          # massa do carro weight=0.37 kg
        self.Mp=0.23;               # massa do pendulo 0.127 / 0.230
        self.l=0.3302;              # comprimento do pendulo 0.1778 / 0.3302
        self.J_p=7.88E-3;           # momento de inércia do pêndulo1.20e-3 / 7.88e-3
        self.Meq = self.Mc+(self.Eff_m*self.kg**2*self.Jm)/self.rp**2
        self.Aeq = (self.Eff_m*self.Eff_g*self.kg*self.kt)/(self.rp*self.Ra)
    def J_Tn(self,x):
        self.Meq = self.Mc+(self.Eff_m*self.kg**2*self.Jm)/self.rp**2
        return(-self.l**2*self.Mp**2*(np.cos(x))**2+
               (self.Mp+self.Meq)*(self.Mp*self.l**2+self.J_p))
    def a1(self,x):
        return(-self.Beq*(self.Mp*self.l**2+self.J_p)/self.J_Tn(x))
    def a2(self,x2,x4):
        return((-self.Bp*self.Mp*self.l*np.cos(x2)-
                self.Mp*self.l*x4*np.sin(x2)*(self.Mp*self.l**2+self.J_p))/self.J_Tn(x2))
    def a3(self,x):
        return(-self.Beq*self.Mp*self.l*np.cos(x)/self.J_Tn(x))
    def a4(self,x2,x4):
        self.Meq = self.Mc+(self.Eff_m*self.kg**2*self.Jm)/self.rp**2
        return((-self.Bp*(self.Meq+self.Mp)-
                self.Mp**2*self.l**2*np.cos(x2)*np.sin(x2)*x4)/self.J_Tn(x2))
    def b1(self,x2):
        self.Aeq = (self.Eff_m*self.Eff_g*self.kg*self.kt)/(self.rp*self.Ra)
        return(self.Aeq*(self.Mp*self.l**2+self.J_p)/self.J_Tn(x2))
    def b2(self,x2):
        self.Aeq = (self.Eff_m*self.Eff_g*self.kg*self.kt)/(self.rp*self.Ra)
        return(self.Aeq*self.Mp*self.l*np.cos(x2)/self.J_Tn(x2))
    def e1(self,x2):
        return(self.Mp**2*self.l**2*self.g*np.sin(x2)*np.cos(x2)/self.J_Tn(x2))
    def e2(self,x2):
        self.Meq = self.Mc+(self.Eff_m*self.kg**2*self.Jm)/self.rp**2
        return(self.Mp*self.l*self.g*np.sin(x2)*(self.Mp+self.Meq)/self.J_Tn(x2))
    def get_K(self):
        # Define o valor de K
        return self.K
    def u(self,y): # Calcula a força de controle
        self.u_ = -np.matmul( self.K , y )#[0]
        if self.u_>=10:
            self.u_=10
        if self.u_<=-10:
            self.u_=-10
        return self.u_
    def MaxOutput(self):
        theta0 = 3*np.pi/180
        x0 = 0
        maxParametros = [ x0, theta0, 0., 0. ]
        maxforca = self.u(maxParametros)
        return maxforca
    def x_dot(self,t,y):
        self.force = self.u(y)
        x_ddot = self.a1(y[1])*y[2]+self.a2(y[1],y[3])*y[3]+self.b1(y[1])*self.force+self.e1(y[1])
        theta_ddot = self.a3(y[1])*y[2]+self.a4(y[1],y[3])*y[3]+self.b2(y[1])*self.force+self.e2(y[1])
        return [ y[2], y[3], x_ddot, theta_ddot]
    def cost_func(self,ti,tf,X_state,U_force):
        self.Q=np.zeros((4,4))
        self.R=0.5 #1
##        self.R=0.75 #2
##        self.Q[0,0],self.Q[1,1]=1,1  # - func1
##        self.Q[0,0],self.Q[1,1]=0.5,1  # - func2
        self.Q[0,0],self.Q[1,1]=0.75,4  # - func3

        Ref = np.array([[0],[0],[0],[0]])
        itae = lambda t,x: np.abs(0-x)*t
        ise = lambda t,x: (0-x)**2
        iae = lambda t,x: np.abs(0-x)
        itse = lambda t,x: t*(0-x)**2
        J = lambda t,x,u: np.matmul(np.transpose(Ref-x),np.matmul(self.Q,Ref-x))+self.R*u**2
##        J = lambda t,x,u: np.matmul(np.transpose(x),np.matmul(self.Q,x))+self.R*u**2
        
##        ITAE,e = quad(itae,ti,tf,args=(X_state[1]))
##        ISE,e = quad(ise,ti,tf,args=(X_state[1]))
##        IAE,e = quad(iae,ti,tf,args=(X_state[1]))
##        ITSE,e = quad(itse,ti,tf,args=(X_state[1]))
        cost,e = quad(J,ti,tf,args=(X_state,U_force))
        return cost#,ITAE,ISE,IAE,ITSE
    def simulation(self,Time,dt,K,X0,W):
        self.tf = Time
        self.K = K
        x_initial = X0
        T = np.linspace(dt,self.tf+dt,int(self.tf/dt))
        initial=0
        data = np.zeros((5,len(T)))
        dataAva = np.zeros((5,len(T)))
        theta,X,forca = [],[],[]
        i=0
        mu, sigma = 0, 0.001
        if W=='y':
            w = lambda mu, sigma: np.random.normal(mu, sigma)
        else:
            w = lambda mu, sigma: 0
        for t in T:
            final = t
            x = [x_initial[0,0],x_initial[1,0],x_initial[2,0],x_initial[3,0]]
            sol = solve_ivp(self.x_dot,[initial,final],x,t_eval=[initial,final],method='RK45')###RK45 alterar
            x_initial = np.array([[sol.y[0,1]+w(mu, sigma)],
                                  [sol.y[1,1]+w(mu, sigma)],
                                  [sol.y[2,1]+w(mu, sigma)],
                                  [sol.y[3,1]+w(mu, sigma)]])#theta'
##            JJ,ITAE,ISE,IAE,ITSE = self.cost_func(initial,final,x_initial,self.u_)
            JJ = self.cost_func(initial,final,x_initial,self.u_)

##            dataAva[0,i] = ITAE
##            dataAva[1,i] = ISE
##            dataAva[2,i] = IAE
##            dataAva[3,i] = ITSE
            dataAva[4,i] = JJ
            
            data[0,i] = self.u_
            data[1,i] = sol.y[0,1]
            data[2,i] = sol.y[1,1]
            data[3,i] = sol.y[2,1]
            data[4,i] = sol.y[3,1]
            i+=1
            initial = final
        return T,data,dataAva
class LINEAR():
    def __init__(self):
        g=9.81;
        Beq=5.4;
        Bp=0.0024;
        kt=0.00767;
        km=0.00767;
        Ra=2.6;
        Jm=3.9E-7;
        rp=6.35E-3;
        kg=3.71
        Eff_m=1;
        Eff_g=1;
        Mc=0.57+0.37;
        Mp=0.23;
        lp=0.3302;
        Jp=7.88E-3;
        Meq = Mc+(Eff_m*kg**2*Jm)/rp**2
        Aeq = (Eff_m*Eff_g*kg*kt)/(rp*Ra)
        self.x1,self.x2,self.x3,self.x4,self.Vm = symbols('x1 x2 x3 x4 Vm')
        a = Mp+Meq
        b = Mp*lp*cos(self.x2)
        c = (Mp*lp**2+Jp)
        d = Mp*lp*sin(self.x2)
        e = Mp*g*lp*sin(self.x2)

        self.x1_dot=self.x3#/(b**2-a*c)

        self.x2_dot=self.x4#/(b**2-a*c)
        # x1  x2   x3   x4
        # xc theta xc' theta'
        self.x3_dot=(c*Beq*self.x3+c*d*self.x4**2-c*Aeq*self.Vm+Bp*b*self.x4-e*b)/(b**2-a*c)

        self.x4_dot=(a*Bp*self.x4-e*a+Beq*self.x3*b+b*d*self.x4**2-b*Aeq*self.Vm)/(b**2-a*c)
    def JACOBIANA(self):
        X = Matrix([self.x1_dot, self.x2_dot, self.x3_dot, self.x4_dot])
        Y = Matrix([self.x1,self.x2,self.x3,self.x4,self.Vm])
        jac = X.jacobian(Y)
        return jac
    def linerizacao(self,OP1,OP2,OP3,OP4,OP5):
        jac = self.JACOBIANA()
        a = jac.evalf(subs={self.x1:OP1,
                            self.x2:OP2,
                            self.x3:OP3,
                            self.x4:OP4,
                            self.Vm:OP5})
        a = np.array(a).astype(np.float64)
        A_op = a[0:4,0:4]
        B_op = a[0:4,4:5]
        return A_op,B_op
class CONTROLE():
    def lqr(self,A,B,Q,R):
        X = np.matrix(scipy.linalg.solve_continuous_are(A, B, Q, R))
        K = np.array(scipy.linalg.inv(R)*(B.T*X))
        eigVals, eigVecs = scipy.linalg.eig(A-B*K)
        return K, X, eigVals
##ln,ctrl = LINEAR(),CONTROLE()
##Q,R=np.zeros((4,4)),np.array([[0.0003*4]])
##Q[0,0]=0.75
##Q[1,1]=4
####
##k=0
##i,j,q,v=0,0,0,0
####A,B = ln.linerizacao(0,i*np.pi/180,q,j,v)
######print(A)
######print(B)
##n=10
##Kmaster = np.zeros((10**2,4))
##d = np.zeros((10**2,4))
##d[:,0] = np.linspace(0,5,100)
##d[:,1] = np.linspace(0,10,100)
##d[:,2] = np.linspace(0,5,100)
##d[:,3] = np.linspace(0,10,100)
##for i in d:
##    A,B = ln.linerizacao(0,i[0]*np.pi/180,i[1],i[2],i[3])
##    K,X,eigVals=ctrl.lqr(A,B,Q,R)
##    print(eigVals)
##    Kmaster[k,:]=K
##    Kmaster[k,0]=round(K[0,0],3)
##    k=k+1
##print(f'K1 std:{np.std(Kmaster[:,0],ddof=1)**2}')
##print(f'K2 std:{np.std(Kmaster[:,1],ddof=1)**2}')
##print(f'K3 std:{np.std(Kmaster[:,2],ddof=1)**2}')
##print(f'K4 std:{np.std(Kmaster[:,3],ddof=1)**2}')
##print('-'*20)
##
##k=0
##i,j,q,v=0,0,0,0
##for i in np.linspace(0,5,n): # graus
##    for j in np.linspace(0,10,n): #rad/s
####    for q in np.linspace(0,5,n):#m/s
####    for v  in np.linspace(0,10,n):#V
##        A,B = ln.linerizacao(0,i*np.pi/180,q,j,v)#[x,theta,x',theta',Vm]
##        K,X,eigVals=ctrl.lqr(A,B,Q,R)
##        Kmaster[k,:]=K
##        Kmaster[k,0]=round(K[0,0],3)
##        k=k+1
##print(f'K1 std:{np.std(Kmaster[:,0])**2}')
##print(f'K2 std:{np.std(Kmaster[:,1])**2}')
##print(f'K3 std:{np.std(Kmaster[:,2])**2}')
##print(f'K4 std:{np.std(Kmaster[:,3])**2}')
##print('-'*20)
##i,j,q,v=0,0,0,0
##k=0
##for i in np.linspace(0,5,n): # graus
####    for j in np.linspace(0,10,n): #rad/s
##    for q in np.linspace(0,5,n):#m/s
####    for v  in np.linspace(0,10,n):#V
##        A,B = ln.linerizacao(0,i*np.pi/180,q,j,v)#[x,theta,x',theta',Vm]
##        K,X,eigVals=ctrl.lqr(A,B,Q,R)
##        Kmaster[k,:]=K
##        Kmaster[k,0]=round(K[0,0],3)
##        k=k+1
##print(f'K1 std:{np.std(Kmaster[:,0])**2}')
##print(f'K2 std:{np.std(Kmaster[:,1])**2}')
##print(f'K3 std:{np.std(Kmaster[:,2])**2}')
##print(f'K4 std:{np.std(Kmaster[:,3])**2}')
##print('-'*20)
##k=0
##i,j,q,v=0,0,0,0
##for i in np.linspace(0,5,n): # graus
####    for j in np.linspace(0,10,n): #rad/s
####    for q in np.linspace(0,5,n):#m/s
##    for v  in np.linspace(0,10,n):#V
##        A,B = ln.linerizacao(0,i*np.pi/180,q,j,v)#[x,theta,x',theta',Vm]
##        K,X,eigVals=ctrl.lqr(A,B,Q,R)
##        Kmaster[k,:]=K
##        Kmaster[k,0]=round(K[0,0],3)
##        k=k+1
##print(f'K1 std:{np.std(Kmaster[:,0])**2}')
##print(f'K2 std:{np.std(Kmaster[:,1])**2}')
##print(f'K3 std:{np.std(Kmaster[:,2])**2}')
##print(f'K4 std:{np.std(Kmaster[:,3])**2}')
##print('-'*20)
##k=0

##i,j,q,v=0,0,0,0
####for i in np.linspace(0,5,n): # graus
##for j in np.linspace(0,10,n): #rad/s
####for q in np.linspace(0,5,n):#m/s
##    for v  in np.linspace(0,10,n):#V
##        A,B = ln.linerizacao(0,i*np.pi/180,q,j,v)#[x,theta,x',theta',Vm]
##        K,X,eigVals=ctrl.lqr(A,B,Q,R)
##        Kmaster[k,:]=K
##        Kmaster[k,0]=round(K[0,0],3)
##        k=k+1
##plt.subplot(2,2,1)
##plt.plot(Kmaster[:,0],'b*')
##plt.ylabel(r'$K_{1}$',fontsize=14)
##plt.xlabel('Partículas',fontsize=14)
##plt.grid()
##plt.subplot(2,2,2)
##plt.plot(Kmaster[:,1],'r*')
##plt.ylabel(r'$K_{2}$',fontsize=14)
##plt.xlabel('Partículas',fontsize=14)
##plt.grid()
##plt.subplot(2,2,3)
##plt.plot(Kmaster[:,2],'g*')
##plt.ylabel(r'$K_{3}$',fontsize=14)
##plt.xlabel('Partículas',fontsize=14)
##plt.grid()
##plt.subplot(2,2,4)
##plt.plot(Kmaster[:,3],'y*')
##plt.ylabel(r'$K_{4}$',fontsize=14)
##plt.xlabel('Partículas',fontsize=14)
##plt.grid()
##
##plt.tight_layout(pad=0.25)
##plt.show()
##print(f'K1 std:{np.std(Kmaster[:,0],ddof=1)**2}')
##print(f'K2 std:{np.std(Kmaster[:,1],ddof=1)**2}')
##print(f'K3 std:{np.std(Kmaster[:,2],ddof=1)**2}')
##print(f'K4 std:{np.std(Kmaster[:,3],ddof=1)**2}')

##Kmaster = np.loadtxt('Kmaster_robusto_LMI_diff_new.dat')

### desenvolvendo um população de ganhos com um modelo diferente
### do modelo a ser controlado

