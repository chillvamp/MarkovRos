


#Now let's open it with pandas
import pandas as pd
from pandas import Series,DataFrame
# Let's import what we'll need for the analysis and visualization
import numpy as np
import matplotlib.pyplot as plt



class node(object):
    def __init__(self,x,y):
        self.x=x
        self.y=y
        
        
class grafo (object):
             def __init__(self,nodos,conec):
                 self.nodos=nodos
                 self.conec=conec        
                 
"""                 
nodos=[]
con=[]         
f  = open ('random_3.top', 'r+')
for line in  f:
    words = line.split()
    if words[1] == "node":
            nodos.append(node(   float(words[3]),float(words[4])  ))
f.close()

        
con =np.ones(len(nodos)*len(nodos)).reshape(len(nodos),len(nodos))*np.inf
f  = open ('random_3.top', 'r+')
for line in  f:
    words = line.split()
    if words[1] == "connection" and float(words[4])   < .2:
        ##print("CONECCCION POSIBLE ENTRE " + words[2] + " Y " + words[3])
        con[int(words[2])] [int(words[3])]=float(words[4])
f.close()        

graphe = grafo(nodos,con)  
"""
A=np.load('A.npy')
nodos=np.load('ccxyth.npy')
con = 1-np.where(A!=0,A,-np.inf)
graphe2=grafo(nodos,con)
 



def dijsktra(nodoinicial,nodofinal,graphe):

##INIT    
    numnodos= len(graphe.nodos)
    con = graphe.conec
    D= np.ones(numnodos)*np.inf
    V= np.zeros(numnodos)
    ruta=[]
    D[nodoinicial]=0
    a = nodoinicial
    ruta.append(a)
 ###FIN INIT   
    
    
    
    
    
    jaja=pd.Series(con[a])
    cona=jaja[jaja != np.inf]
    V[a]=1
    for c in cona.index:
        if cona [c] < D[c]:
            D[c] = cona[c]
            Daux = pd.Series(D)
    a= Daux[V!=1].argmin()
    ruta.append(a)
#####FF
    while a != nodofinal:
        jaja=pd.Series(con[a])
        cona=jaja[jaja != np.inf]
        V[a]=1
        for c in cona.index:
            if cona[c]+D[a]  < D[c] :
                D[c] = cona[c] + D[a]
        Daux= pd.Series(D)
        a= Daux[V!=1].argmin()
        ruta.append(a)
    
    
    
    
    
    
    
    return (ruta)

print ( dijsktra(13,67,graphe2))




















