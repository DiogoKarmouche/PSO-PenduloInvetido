# PSO-PenduloInvetido
Classes e programações utilizadas para a simulação do sistema pêndulo invertido e controle via o algoritmo PSO

## Teste das matrizes Q e R
Para realizar o teste de das matrizes Q e R, na classe ControlePenduloInvertido.py, na função cost_func comentar e descomentar as matrizes Q e R que seja desejado testar, em seguida utilizar o seguinte código:
import numpy as np
from ControlePenduloInvertido import *
ctrl = CONTROLE()
Q,R=np.zeros((4,4)),np.array([[0.0003*4]])
Q[0,0]=0.75
Q[1,1]=4
K,X,eigVals=ctrl.lqr(A,B,Q,R)
sis = Sistema()          # inicializa um pêndulo invertido em espaço de estados
tempo,dt = 10,0.01          # tempo de simulação do sistema dinâmico
theta0,x0 = 3*np.pi/180,0
x_initial = np.array([[0],[3*np.pi/180],[0],[0]])
T,data,dataAva = sis.simulation(Time=tempo,dt=dt,K = K,X0=x_initial,W='n')
CE = np.sum(np.abs(data[0,:]))
ISE = np.sum(dataAva[1,:])
