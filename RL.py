# -*- coding: utf-8 -*-
"""
Created on Tue Mar 28 08:31:41 2017

@author: 90483
"""
import traci
import random as rd
import timeit
import numpy as np
import copy
import _c # user define constant

def readTraingingSettings():
    with open('map_settings.txt') as f:
        lines = f.readlines()
    mapName=lines[0].split()
    TCid=lines[1].split()
    laneSet=lines[2].split() # lane set
    laneSigDigit=lines[3].split() # each lane's traffic signal index 
    laneSigDigit=map(int,laneSigDigit) # string to integer
    phase_set=lines[4].split() # action set (each string represents a phase)
    y_set=lines[5].split() 
    minG=lines[6].split() # minimun green time
    minG=map(int,minG) # string to integer
    
    return mapName[0],TCid[0],laneSet,laneSigDigit,phase_set,y_set,minG


def calOBJ():
    # weighted average of N_q and N_er
    return -1*( sum(s_q) + np.dot(s_q,s_er)) # assume weight =1

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
        print "random pick."
        return rd.choice(a)
    else: # find the action that has the minimum cost
        # Snew * w in R^│A│ each is the Q value under (s,a)
        Q=np.dot(w.T,Snew)
        print "Q(Snew) = ",Q
        return np.argmax(Q)
    
def updatePhase(curPhase,g_elps):
    # add yello and all-red phase to Plist 
    ar='r'*len(phase_set[0]) 
    y=2 # yello 
    r=1 # all red
    if not Anew: # Anew == 0 means phase changing
        for i in range(y):
            Plist.append(y_set[curPhase])
        for i in range(r):
            Plist.append(ar)
        curPhase+=1
        curPhase = curPhase % len(phase_set)
        Plist.append(phase_set[curPhase])
        g_elps=0
    return curPhase,Plist,g_elps

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
    
#     # Network settings
    start = timeit.default_timer()
    mapName,TCid,laneSet,laneSigDigit,phase_set,y_set,minG=readTraingingSettings()
    passable_sig=['g','G','y'] # signals that resets elapsed time
#   
#    # Variable initialization
    A_set=[0,1] # stop current phase or extend it
    s_q=np.zeros(len(laneSet)) # number of stopping veh on each lane
    s_er=np.zeros(len(laneSet)) # elapsed red time of each lane
    s_eg=np.zeros(len(laneSet)) # elapsed green time of each lane, size L
    perf_lane=np.zeros(len(laneSet)) # number of stopped vehicle through simulation period (T*L matrix)
    w=np.zeros((len(laneSet)+len(phase_set),len(A_set))) # parameters of linear approximation, (L+P)*A matrix. P is one-hot
    
    if not _c.b_train: # load existing training result and start to simulate
        w=np.load('training_result')
    # A Q(s,a) value can be estimated by w[:,a]*[s_q]
    if _c.b_outrlt and _c.b_train:
        frlt = open('training_result.txt', 'w')   

    A=0 # action (extend or stop the current phase)
    Anew=A # new action (0 or 1)
    Plist=[] # list of phases storing y_set and all red, will pop through time if the list is not empty
    t=0 # step
    R=0 # total delay
    D=0 # departure veh
    curPhase=0 # current phase
    g_elps=0
    
    # initial state = [s_q,P_onehot]
    S=np.append(s_q,np.zeros(len(phase_set)))
    S[len(laneSet)+curPhase]=1 # initial phase =1 others =0 
    Snew=copy.copy(S)
    
    # Start simulation
    if _c.b_train:
#         traci.start(["sumo", "-c", mapName+".sumo.cfg"])
         traci.start(["sumo-gui", "-c", mapName+".sumo.cfg","--random"])
    else:
         traci.start(["sumo-gui", "-c", mapName+".sumo.cfg","--random"])
    try:
        while t<_c.T:
             # get current simulation time
             ct=traci.simulation.getCurrentTime()/1000
             # Update state
             s_q,s_er,s_eg=updateS() # since Snew=s_q, the func updates both
             R=R+calOBJ()
             
             # still in phase transition(lost time), pop up a plan index each time
             if len(Plist)>0: 
                 traci.trafficlights.setRedYellowGreenState(TCid,Plist.pop(0))
             
             # take action and update weight
             if t%_c.updt_freq==0 and g_elps>minG[curPhase]:
                 Snew=np.append(s_q,np.zeros(len(phase_set))) # new state
                 Snew[len(laneSet)+curPhase]=1 # current state =1 others =0 
                 print "t = ",ct,"\nS = ",S,"\nA = ",A,"\nG_elps = ",g_elps,"\nR = ",R,"\nSnew = ",Snew
                 # Choose an action Anew
                 Anew,e=choseAction()
                 curPhase,Plist,g_elps=updatePhase(curPhase,g_elps)
                 # Update w
                 if _c.b_train:
                     w=updateW()
                     
                 print "Anew (e = ",e,") = ",Anew,"\nw = \n",w,"\n\n"
                 S=copy.copy(Snew)
                 A=Anew
                 R=0
                 D=0
                 
             perf_lane=np.vstack((perf_lane,s_q))
             t+=1
             
             traci.simulationStep()
             g_elps+=1
             
        if _c.b_outrlt and _c.b_train:
            frlt.write(str(w))
            w.dump('training_result')
            frlt.close()
        if _c.b_outPerf:
            np.savetxt("sim_perf.csv", perf_lane,fmt='%03d', delimiter=",")
        stop = timeit.default_timer()
        print "computing time = ", stop-start
        traci.close()
    except:
        if _c.b_outPerf:
            np.savetxt("sim_perf.csv", perf_lane, fmt ='%03d', delimiter=",")
   
