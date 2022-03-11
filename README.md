# PSO-PenduloInvetido
Classes e programações utilizadas para a simulação do sistema pêndulo invertido e controle via o algoritmo PSO

## Teste das matrizes Q e R
Para realizar o teste de das matrizes Q e R, na classe ControlePenduloInvertido.py, na função cost_func comentar e descomentar as matrizes Q e R que seja desejado testar, em seguida utilizar o seguinte código:<br />
<br />
import numpy as np<br />
from ControlePenduloInvertido import *<br />
<br />
ctrl = CONTROLE()<br />
Q,R=np.zeros((4,4)),np.array([[0.0003*4]])<br />
Q[0,0]=0.75<br />
Q[1,1]=4<br />
K,X,eigVals=ctrl.lqr(A,B,Q,R)<br />
sis = Sistema() <p>         # inicializa um pêndulo invertido em espaço de estados<br />
tempo,dt = 10,0.01 <p>         # tempo de simulação do sistema dinâmico<br />
theta0,x0 = 3*np.pi/180,0<br />
x_initial = np.array([[0],[3*np.pi/180],[0],[0]])<br />
T,data,dataAva = sis.simulation(Time=tempo,dt=dt,K = K,X0=x_initial,W='n')<br />
CE = np.sum(np.abs(data[0,:]))<br />
ISE = np.sum(dataAva[1,:])<br />
