#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Robótica Computacional - 
# Grado en Ingeniería Informática (Cuarto)
# Práctica: Resolución de la cinemática inversa mediante CCD
#           (Cyclic Coordinate Descent).

import math
import time
import sys
from math import *
import numpy as np
import matplotlib.pyplot as plt
import colorsys as cs

# ******************************************************************************
# Declaración de funciones

def muestra_origenes(O,final=0):
  # Muestra los orígenes de coordenadas para cada articulación
  print('Origenes de coordenadas:')
  for i in range(len(O)):
    print('(O'+str(i)+')0\t= '+str([round(j,3) for j in O[i]]))
  if final:
    print('E.Final = '+str([round(j,3) for j in final]))

def muestra_robot(O,obj):
  # Muestra el robot graficamente
  plt.figure()
  plt.xlim(-L,L)
  plt.ylim(-L,L)
  T = [np.array(o).T.tolist() for o in O]
  for i in range(len(T)):
    plt.plot(T[i][0], T[i][1], '-o', color=cs.hsv_to_rgb(i/float(len(T)),1,1))
  plt.plot(obj[0], obj[1], '*')
  plt.pause(0.0001)
  plt.show()
  
#  input()
  plt.close()

def matriz_T(d,th,a,al):
  # Calcula la matriz T (ángulos de entrada en grados)
  
  return [[cos(th), -sin(th)*cos(al),  sin(th)*sin(al), a*cos(th)]
         ,[sin(th),  cos(th)*cos(al), -sin(al)*cos(th), a*sin(th)]
         ,[      0,          sin(al),          cos(al),         d]
         ,[      0,                0,                0,         1]
         ]

def cin_dir(th,a):
  #Sea 'th' el vector de thetas
  #Sea 'a'  el vector de longitudes
  T = np.identity(4)
  o = [[0,0]]
  for i in range(len(th)):
    T = np.dot(T,matriz_T(0,th[i],a[i],0))
    tmp=np.dot(T,[0,0,0,1])
    o.append([tmp[0],tmp[1]])
  return o

def camino_corto(diferencia):
  if (diferencia > pi): 
    return (diferencia - 2*pi)
  elif(diferencia < pi*-1):
    return (diferencia + 2*pi)
  return diferencia
  
def maximo_angulo(limiteAngulo,angulo):
  #print("Ángulo en radianes",angulo)
  # Opcion que indica que no tiene limite de movimiento
  if(limiteAngulo == 0):
    return angulo
  else:
    limite_rad = math.radians(limiteAngulo)
    # Caso ángulo negativo
    if(angulo < 0):
      limite_rad = limite_rad * -1
      #print("Limite en radianes",limite_rad)
      # Ángulo mayor que limite (como es negativo se devuelve el ángulo)
      if(limite_rad < angulo):
        return angulo
      # Ángulo menor que limite (como es negativo se devuelve el límite)
      else:
        return limite_rad
    else:
      #print("Limite en radianes",limite_rad)
      if(limite_rad < angulo):
        return limite_rad
      else:
        return angulo
  

# ******************************************************************************
# Cálculo de la cinemática inversa de forma iterativa por el método CCD

# valores articulares arbitrarios para la cinemática directa inicial
th=[0.,0.,0.]
a =[5.,5.,5.]
L = sum(a) # variable para representación gráfica
EPSILON = .01

plt.ion() # modo interactivo

# introducción del punto para la cinemática inversa
if len(sys.argv) != 3:
  sys.exit("python " + sys.argv[0] + " x y")
objetivo=[float(i) for i in sys.argv[1:]]
O=cin_dir(th,a)
#O=zeros(len(th)+1) # Reservamos estructura en memoria
 # Calculamos la posicion inicial
print ("- Posicion inicial:")
muestra_origenes(O)

limiteAngulo = 90

dist = float("inf")
prev = 0.
iteracion = 1
while (dist > EPSILON and abs(prev-dist) > EPSILON/100.):
  prev = dist
  O=[cin_dir(th,a)]
  # Para cada combinación de articulaciones:

  for i in range(len(th)):
    # cálculo de la cinemática inversa:
    #             Objetivo      Punto de la articulacion iteracion
    tan1 = atan2(objetivo[1] - O[i][len(th)-1-i][1],
                 objetivo[0] - O[i][len(th)-1-i][0])

    #           Último punto        Punto de la articulacion iteracion
    tan2 = atan2(O[i][len(th)][1] - O[i][len(th)-1-i][1],
                 O[i][len(th)][0] - O[i][len(th)-1-i][0])
    
    diferencia = tan1-tan2
    if((th[len(th)-1-i] + maximo_angulo(limiteAngulo,camino_corto(diferencia))) > math.radians(limiteAngulo)):
      th[len(th)-1-i] = math.radians(limiteAngulo)
    elif((th[len(th)-1-i] + maximo_angulo(limiteAngulo,camino_corto(diferencia))) < -math.radians(limiteAngulo)):
      th[len(th)-1-i] = -(math.radians(limiteAngulo))
    th[len(th)-1-i] += maximo_angulo(limiteAngulo,camino_corto(diferencia))
    print("Tetha: ",maximo_angulo(limiteAngulo,camino_corto(diferencia)))
    #print("Valor del incremento: ",tan1-tan2)
    O.append(cin_dir(th,a))

  dist = np.linalg.norm(np.subtract(objetivo,O[-1][-1]))
  print ("\n- Iteracion " + str(iteracion) + ':')
  muestra_origenes(O[-1])
  muestra_robot(O,objetivo)
  print ("Distancia al objetivo = " + str(round(dist,5)))
  iteracion+=1
  O[0]=O[-1]
  time.sleep(0.5)

if dist <= EPSILON:
  print ("\n" + str(iteracion) + " iteraciones para converger.")
else:
  print ("\nNo hay convergencia tras " + str(iteracion) + " iteraciones.")
print ("- Umbral de convergencia epsilon: " + str(EPSILON))
print ("- Distancia al objetivo:          " + str(round(dist,5)))
print ("- Valores finales de las articulaciones:")
for i in range(len(th)):
  print ("  theta" + str(i+1) + " = " + str(round(th[i],3)))
for i in range(len(th)):
  print ("  L" + str(i+1) + "     = " + str(round(a[i],3)))
