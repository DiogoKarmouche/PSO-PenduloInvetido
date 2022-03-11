# PSO-PenduloInvetido
Classes e programações utilizadas para a simulação do sistema pêndulo invertido e controle via o algoritmo PSO

## Teste das matrizes Q e R para a função de custo
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
## Teste para definição de máximos e mínimos dos pontos de operação
import numpy as np<br />
import matplotlib.pyplot as plt
from ControlePenduloInvertido import *<br />
<br />
K = np.array([0,0,0,0])
sis = Sistema() <p>         # inicializa um pêndulo invertido em espaço de estados<br />
tempo,dt = 3,0.01 <p>         # tempo de simulação do sistema dinâmico<br />
theta0,x0 = 3*np.pi/180,0<br />
x_initial = np.array([[0],[3*np.pi/180],[0],[0]])<br />
T,data,dataAva = sis.simulation(Time=tempo,dt=dt,K = K,X0=x_initial,W='n')<br />
fig, ax1 = plt.subplots()<br />
ax1.set_ylabel(r'$\dot{x}_{c}$', color='red')<br />
ax1.plot(T, data[3,:],'r-')<br />
ax2 = ax1.twinx()<br />
ax2.set_ylabel(r'$\dot{\theta}$', color='blue')<br />
ax2.plot(T, data[4,:],'b-')<br />
plt.show()<br />

## Simulação do PSO

  
