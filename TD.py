# -*- coding: utf-8 -*-
"""
Created on Tue Mar 28 08:31:41 2017

@author: 90483
"""
import traci
import random as rd
import numpy as np
import copy
import _c # user define constant

def readTraingingSettings(fn):
    with open('training_settings.txt') as f:
        lines = f.readlines()
    mapName=lines[0].split()
    TCid=lines[1].split()
    laneSet=lines[2].split()
    laneSigDigit=lines[3].split()
    laneSigDigit=map(int,laneSigDigit) # string to integer
    A_set=lines[4].split()
    y_set=lines[5].split()
    
    return mapName[0],TCid[0],laneSet,laneSigDigit,A_set,y_set


def calOBJ():
    # weighted average of N_q and N_er
    return -1*sum(s_q) # assume weight =1

def choseAction():
    # given state, return an action with e-greedy policy
    # decent e rate, e = max (min_p,max_p-t/max_ite)
    # thus e = min_p when t =(maxp-minp)*maxite
    if _c.b_train:
        e = max(_c.e_min_p,_c.e_max_p-t/_c.e_max_ite)
    else:
        e=0 # deterministic
    new_p=e_greedy(e) # get new phase index
    return new_p,e

def e_greedy(e):
    # episolon greedy
    # return phase number (0,1,...,P-1)

    if rd.random()<e: # random pick
        a=range(len(A_set))
        if _c.b_outlog:
            fout.write("\nrandom pick.")
        print "random pick."
        return rd.choice(a)
    else: # find the action that has the minimum cost
        # Snew * w in R^│A│ each is the Q value under (s,a)
        Q=np.dot(w.T,Snew)
        if _c.b_outlog:
            fout.write("\nQ(Snew) = "+str(Q))
        print "Q(Snew) = ",Q
        return np.argmax(Q)
    
def addLostTime():
    # add yello and all-red phase to Plist 
    ar='r'*len(A_set[0]) 
    y=2 # yello 
    r=1 # all red
    if A!=Anew: # phase changed
        for i in range(y):
            Plist.append(y_set[A])
        for i in range(r):
            Plist.append(ar)
        Plist.append(A_set[Anew])
    return Plist

def updateS():
    #update current queue length and elapsed time
    # get signal state
    sig_stat=traci.trafficlights.getRedYellowGreenState(TCid)
    for l in laneSet:
        lind=laneSet.index(l) # lane index
        sind=laneSigDigit[lind] #lane sig index
        # update elapsed time
        if sig_stat[sind] in passable_sig:
            s_er[lind]=0 # elapsed time = 0 if tl turns to green
            s_eg[lind]+=1 # elapsed time = 0 if tl turns to green
        else:
            s_er[lind]+=1
            s_eg[lind]=0
        # update queue length
        s_q[lind]=traci.lane.getLastStepHaltingNumber(l)
    return s_q,s_er,s_eg

def updateW():
#    inc=R + beta*np.dot(S_new_a,w_a)-np.dot(S_a,w_a)
#    print "inc=",inc
    if _c.updateScheme=="Q":
        inc=R + _c.beta*np.dot(Snew,w[:,e_greedy(0)])-np.dot(S,w[:,A])
    elif _c.updateScheme=="SARSA":
        inc=R + _c.beta*np.dot(Snew,w[:,Anew])-np.dot(S,w[:,A])
    w[:,A]=w[:,A]+_c.alpha * inc * S
#    w[:,A]=w[:,A]+alpha * inc * S.reshape(len(S),1)
#    w = w + alpha*(R + beta*w*Snew-w*S)*S
    return w

if __name__ == "__main__":
    # Network settings
    mapName,TCid,laneSet,laneSigDigit,A_set,y_set=readTraingingSettings("training_settings.txt")
    passable_sig=['g','G','y'] # signals that resets elapsed time
   
    # Variable initialization
    s_q=np.zeros(len(laneSet)) # number of stopping veh on each lane
    s_er=np.zeros(len(laneSet)) # elapsed red time of each lane
    s_eg=np.zeros(len(laneSet)) # elapsed green time of each lane, size L
    w=np.zeros((len(laneSet),len(A_set))) # parameters of linear approximation, L*A matrix. 
    if not _c.b_train: # load existing training result and start to simulate
        w=np.load('training_result')
    # A Q(s,a) value can be estimated by w[:,a]*[s_q]
    if _c.b_outlog and _c.b_train:
        fout = open('training_log.txt', 'w')   
    if _c.b_outrlt and _c.b_train:
        frlt = open('training_result.txt', 'w')   
    A=0 # action (index of phase)
    Anew=A # new action
    Plist=[] # list of phases storing y_set and all red, will pop through time if the list is not empty
    S=copy.copy(s_q) # state
    Snew=s_q # new state
    t=0 # step
    R=0 # total delay
    D=0 # departure veh
    
    # Start simulation
#    traci.start(["sumo-gui", "-c", mapName+".sumo.cfg"])
    traci.start(["sumo", "-c", mapName+".sumo.cfg"])
    while t<_c.T:
         # get current simulation time
         ct=traci.simulation.getCurrentTime()/1000
         # Update state
         s_q,s_er,s_eg=updateS() # since Snew=s_q, the func updates both
         R=R+calOBJ()
         
         if len(Plist)>0: # lost time, pop up a plan index each time
             traci.trafficlights.setRedYellowGreenState(TCid,Plist.pop(0))
         if t%_c.updt_freq==0:
             print "t = ",ct,"\nS = ",S,"\nA = ",A,"\nR = ",R,"\nSnew = ",Snew
             if _c.b_outlog:
                 fout.write("t = "+str(ct)+"\nS = "+str(S)+"\nA = "+str(A)+"\nR = "+str(R)+"\nSnew = "+str(Snew))
             # Choose an action Anew
             Anew,e=choseAction()
             if _c.b_outlog:
                 fout.write("\nAnew (e = "+str(e)+") = "+str(Anew))
             Plist=addLostTime()
             # Update w
             if _c.b_train:
                 w=updateW()
             if _c.b_outlog and _c.b_train:
                 fout.write("\nw = \n"+str(w)+"\n\n")

             print "Anew (e = ",e,") = ",Anew,"\nw = \n",w,"\n\n"
             S=copy.copy(Snew)
             A=Anew
             R=0
             D=0
         t+=1
         traci.simulationStep()
         
    if _c.b_outlog and _c.b_train:
        fout.close()
    if _c.b_outrlt and _c.b_train:
        frlt.write(str(w))
        w.dump('training_result')
        frlt.close()
traci.close()
