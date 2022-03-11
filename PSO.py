from __future__ import division
import random
import math
import matplotlib.pyplot as plt
import numpy as np
import PenduloInvertidoControle as ss
from time import time

# cria uma classe Partícula
class Particle:
    def __init__(self,x0):
        self.w=0.9                  # constant inertia weight (how much to weigh the previous velocity)
        self.position_i=[]          # particle position
        self.velocity_i=[]          # particle velocity
        self.pos_best_i=[]          # best position individual
        self.err_best_i=-1          # best error individual
        self.err_i=-1               # error individual

##      gera uma particula e sua posição e velocidade nas dimensões determinadas
        for i in range(0,num_dimensions):
##            self.velocity_i.append(0)# particulas começam paradas
            self.velocity_i.append(random.uniform(-1,1))# gera velocidades aleatórias
            self.position_i.append(round(x0[i],4))# posiciona as posições da particula no espaço
            
##      evaluate current fitness
    def evaluate(self,costFunc):
        self.err_i=costFunc(self.position_i)
##      check to see if the current position is an individual best
        if self.err_i < self.err_best_i or self.err_best_i==-1:
            self.pos_best_i=self.position_i
            self.err_best_i=self.err_i
##     update new particle velocity
    def update_velocity(self,pos_best_g):
        c1=2                        # cognative constant
        c2=1                        # social constant
        for i in range(0,num_dimensions):
            r1=random.uniform(0,1)
            r2=random.uniform(0,1)
            vel_cognitive=c1*r1*(self.pos_best_i[i]-self.position_i[i])
            vel_social=c2*r2*(pos_best_g[i]-self.position_i[i])
            self.velocity_i[i]=(self.w*self.velocity_i[i]+vel_cognitive+vel_social)
##     update the particle position based off new velocity updates
    def update_position(self,bounds):
        for i in range(0,num_dimensions):
            k=100
            vel_max = (bounds[i][1]-bounds[i][0])/k
            if self.velocity_i[i]>vel_max:
                self.velocity_i[i]=vel_max
                
            if self.velocity_i[i]<-vel_max:
                self.velocity_i[i]=-vel_max
            
            self.position_i[i]=self.position_i[i]+self.velocity_i[i]
            
            # adjust maximum position if necessary
            if self.position_i[i]>bounds[i][1]:
                self.position_i[i]=bounds[i][1]
            # adjust minimum position if neseccary
            if self.position_i[i] < bounds[i][0]:
                self.position_i[i]=bounds[i][0]

class PSO():
    def __init__(self,x0,bounds,num_particles,maxiter=100):
##      Inicializa o programa
        global num_dimensions,timeLog,k1,k2,k3,k4,J_swarm,k1_best,k2_best,k3_best,k4_best,intposi
        err_best_g=-1
        self.i=0
        self.np=0
        self.dados = np.zeros((5,int(10/0.01),num_particles,maxiter))
        
##      gurda a progressão dos melhores K
        k1_best,k2_best,k3_best,k4_best=[],[],[],[]
        k1=np.zeros((num_particles,maxiter))
        k2=np.zeros((num_particles,maxiter))
        k3=np.zeros((num_particles,maxiter))
        k4=np.zeros((num_particles,maxiter))
        timeLog = []
        J_swarm = np.zeros((num_particles,maxiter))
        
##      --- COST FUNCTION 
##      function we are attempting to optimize (minimize)
        sis = ss.Sistema()          # inicializa um pêndulo invertido em espaço de estados
        tempo,dt = 10,0.01          # tempo de simulação do sistema dinâmico
##      função a ser avaliada  
        def func(x):
##          valores iniciais
            theta0,x0 = 3*np.pi/180,0
            x_initial = np.array([[0],[3*np.pi/180],[0],[0]])
            K = np.zeros(4)
            K[0]=x[0]
            K[1]=x[1]
            K[2]=x[2]
            K[3]=x[3]
            T,data,dataAva = sis.simulation(Time=tempo,dt=dt,K = K,X0=x_initial,W='n') # simulação
            self.dados[:,:,self.np,self.i] = data

            total = np.sum(dataAva[4,:])
            return total
##      função para gerar as particulas
        def SwarmGeneration(num_particles):
            swarm=[]
            for i in range(0,num_particles):
                swarm.append(Particle(x0[i]))
            return swarm
        def SwarmAvaliation(swarm,
                            func,
                            num_particles,
                            pos_best_g,err_best_g):
            for j in range(0,num_particles):
                self.np=j
                swarm[j].evaluate(func)
                k1[j,self.i]=swarm[j].position_i[0]#salva os valores em um matriz
                k2[j,self.i]=swarm[j].position_i[1]#   ''
                k3[j,self.i]=swarm[j].position_i[2]#   ''
                k4[j,self.i]=swarm[j].position_i[3]#   ''
                J_swarm[j,self.i]=swarm[j].err_i
##              determine if current particle is the best (globally)
                if swarm[j].err_i < err_best_g or err_best_g == -1:
                    pos_best_g=list(swarm[j].position_i)
                    err_best_g=float(swarm[j].err_i)
                    print('########################')
                    print(f'Swarm Best:{pos_best_g}, {err_best_g}')
                    print('########################')
                    pass
                pass
            err_best_g = func(pos_best_g) #verificação de melhor dos melhores,
                                          #para verifcar se condições nao foram alteradas 
            o=0
            for i in J_swarm[:,self.i]:
                if i<err_best_g:
                    err_best_g=i
                    pos_best_g=[k1[o,self.i],k2[o,self.i],k3[o,self.i],k4[o,self.i]]
                    print('Provavel mudança em ambiente')
                o+=1
                pass
            print(f'Swarm Best:{pos_best_g}, {err_best_g}')
            return err_best_g,pos_best_g
        def SwarmUpdate(swarm,num_particles,pos_best_g,w):
            for j in range(0,num_particles):
                swarm[j].update_velocity(pos_best_g)
                swarm[j].update_position(bounds)
            return swarm

##      determina a dimensão do espaço de busca
        num_dimensions=np.shape(x0)[1]
        err_best_g=-1                   # best error for group
        pos_best_g=[]                   # best position for group
##      establish the swarm
        swarm = SwarmGeneration(num_particles)
        self.i,interacao=0,[]
##      begin optimization loop
        error=np.zeros(maxiter)
        i=0
        while self.i < maxiter:
            t1 = time()
            print(f'Iteração: {self.i}')

##          cycle through particles in swarm and evaluate fitness
            err_best_g,pos_best_g = SwarmAvaliation(swarm,func,num_particles,pos_best_g,err_best_g)

##          cycle through swarm and update velocities and position
            swarm = SwarmUpdate(swarm,num_particles,pos_best_g,swarm[0].w)
            error[self.i] = err_best_g
            print(f'Melhor Custo do swarm: {error[self.i]}')
            print(f'Melhor K do swarm: {pos_best_g}')
          # ~~~~~~~Inserção da mudança~~~~~~~ #
            if self.i==49:
                # atrito
##                sis.Bp=sis.Bp*1.1
##                sis.Beq=sis.Beq*1.1
##                print(f'Bp: {sis.Bp}, Beq: {sis.Beq}')
                # massa
                sis.Mc=sis.Mc-sis.Mc*0.2
                print(f'M: {sis.Mc}')
                pass
            if self.i==99:
                # atrito
##                sis.Bp=sis.Bp*1.1
##                sis.Beq=sis.Beq*1.1
##                print(f'Bp: {sis.Bp}, Beq: {sis.Beq}')
##                comprimentoOriginal = sis.l
##                sis.l=sis.l-(comprimentoOriginal/3)
##                print(f'L: {sis.l}')
                # massa
                sis.Mc=sis.Mc-sis.Mc*0.2
                print(f'M: {sis.Mc}')
                pass
            if self.i==149:
                # atrito
##                sis.Bp=sis.Bp*1.1
##                sis.Beq=sis.Beq*1.1
##                print(f'Bp: {sis.Bp}, Beq: {sis.Beq}')
                # massa
                sis.Mc=sis.Mc-sis.Mc*0.2
                print(f'M: {sis.Mc}')
                pass
            if self.i==199:
                # atrito
##                sis.Bp=sis.Bp*1.1
##                sis.Beq=sis.Beq*1.1
##                print(f'Bp: {sis.Bp}, Beq: {sis.Beq}')
##                sis.l=sis.l-(comprimentoOriginal/3)
##                print(f'L: {sis.l}')
                # massa
                sis.Mc=sis.Mc-sis.Mc*0.2
                print(f'M: {sis.Mc}')
                pass
            t2 = time()
            print('Tempo da iteração ',t2-t1)
          # ~~~~~~~Inserção da mudança~~~~~~~ #
            
            k1_best.append(pos_best_g[0])
            k2_best.append(pos_best_g[1])
            k3_best.append(pos_best_g[2])
            k4_best.append(pos_best_g[3])
            print('Tempo passado',t2-t1)
            timeLog.append(t2-t1)
            self.i+=1
            pass
        # print final results
        print(f'Melhor K: {pos_best_g}')
        
if __name__ == "__PSO__":
    main()
