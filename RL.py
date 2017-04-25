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
    lostTime=lines[7].split() # lost time [yellow, red]
    lostTime=map(int,lostTime) # string to integer
    return mapName[0],TCid[0],laneSet,laneSigDigit,phase_set,y_set,minG,lostTime[0],lostTime[1]


def calOBJ(laneSet,s_q,s_er):
    # weighted average of N_q and N_er
    delay=sum(s_q)
    penal=0 # penaty value
    for er in s_er:
        exced=max(er-_c.tol_er,0) # the time over tolerance level
        if exced>0:
            penal=+ exced*_c.pen_rate
    return -1*( delay + penal ) 

def choseAction():
    # given state, return an action with e-greedy policy
    # decent e rate, e = max (min_p,max_p-t/max_ite)
    # thus e = min_p when t =(maxp-minp)*maxite
    if _c.b_train:
        e = max(_c.e_min_p,_c.e_max_p-t * (_c.e_max_p-_c.e_min_p) /_c.e_max_ite)
    else:
        e=0 # deterministic
    new_p,Q=e_greedy(e) # get new phase index
    return Q,new_p,e

def e_greedy(e):
    # episolon greedy
    # return phase number (0,1,...,P-1)
    # Snew * w in R^│A│ each is the Q value under (s,a)
    W=np.c_[Wc,We[:,curPhase]] # weight representing changing or extending current phase
    Q=np.dot(W.T,Snew)
    Q[curPhase]=-float("inf")  # can't compare to the curent phase
    if rd.random()<e: # random pick
        a=range(len(A_set))
        a.pop(curPhase)
        
        print "random pick."
        return rd.choice(a),Q
    else: # find the action that has the minimum cost
        print "Q(Snew) = ",Q
        return np.argmax(Q),Q
    
def updatePhase(curPhase,g_elps):
    # add yello and all-red phase to Plist 
    ar='r'*len(phase_set[0]) 
    if Anew < N_p: # Anew < Np means phase changing
        for i in range(ls_y): # lost time yellow
            Plist.append(y_set[curPhase])
        for i in range(ls_r): # lost time red
            Plist.append(ar)
        curPhase = Anew
        Plist.append(phase_set[curPhase])
        g_elps=0
    return curPhase,Plist,g_elps

def updateS(s_q,s_er,s_eg):
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
    Qact=e_greedy(0)
    if Qact==N_p:
       Qw=copy.copy(We)
    else:
       Qw=copy.copy(Wc)
       
    # update learning rate
    alpha =max(_c.alpha_min,_c.alpha_max - t * (_c.alpha_max - _c.alpha_min)/_c.alpha_maxite)
    print "Learning rate = ", alpha
    if A==N_p and Anew==N_p: # (extend, extend) update We[:,curPhase] using We[:,curPhase] We[:,curPhase]
        if _c.updateScheme=="Q":
            inc=R + _c.beta * np.dot(Snew,Qw[:,curPhase]) - np.dot(S,We[:,curPhase]) 
        elif _c.updateScheme=="SARSA":
            inc=R + _c.beta * np.dot(Snew,We[:,curPhase]) - np.dot(S,We[:,curPhase]) 
        We[:,curPhase]=We[:,curPhase] + alpha * inc * S
    elif A==N_p and Anew < N_p: # (extend, change) update We[:,lastPhase] using Wc[:,curPhase] We[:,lastPhase]
        if _c.updateScheme=="Q":
            inc=R + _c.beta * np.dot(Snew,Qw[:,curPhase]) - np.dot(S,We[:,lastPhase]) 
        elif _c.updateScheme=="SARSA":
            inc=R + _c.beta * np.dot(Snew,Wc[:,curPhase]) - np.dot(S,We[:,lastPhase]) 
        We[:,lastPhase]=We[:,lastPhase] + alpha * inc * S
    elif A < N_p and Anew==N_p: # (change, extend) update Wc[:,curPhase] using We[:,curPhase] Wc[:,curPhase]
        if _c.updateScheme=="Q":
            inc=R + _c.beta * np.dot(Snew,Qw[:,curPhase]) - np.dot(S,Wc[:,curPhase]) 
        elif _c.updateScheme=="SARSA":
            inc=R + _c.beta * np.dot(Snew,We[:,curPhase]) - np.dot(S,Wc[:,curPhase]) 
        Wc[:,curPhase]=Wc[:,curPhase] + alpha * inc * S
    elif A < N_p and Anew < N_p: # (change, change) update Wc[:,lastPhase] using Wc[:,curPhase] Wc[:,lastPhase]
        if _c.updateScheme=="Q":
            inc=R + _c.beta * np.dot(Snew,Qw[:,curPhase]) - np.dot(S,Wc[:,lastPhase]) 
        elif _c.updateScheme=="SARSA":
            inc=R + _c.beta * np.dot(Snew,Wc[:,curPhase]) - np.dot(S,Wc[:,lastPhase]) 
        Wc[:,lastPhase]=Wc[:,lastPhase] + alpha * inc * S
        
    return Wc,We

if __name__ == "__main__":
    # Read Network settings
    start = timeit.default_timer()
    mapName,TCid,laneSet,laneSigDigit,phase_set,y_set,minG,ls_y,ls_r=readTraingingSettings()
    passable_sig=['g','G','y'] # signals that resets elapsed time
    N_p=len(phase_set) # number of phases
    N_l=len(laneSet) # number of lanes
    
    # Variable initialization
    A_set=range(N_p+1) # [switch to a phase (phase index), extend current phase (last column)]
    We=np.zeros((N_l*2,N_p)) # weight of extending current phase  
    Wc=np.zeros((N_l*2,N_p)) # weight of changing to otehr phase  
    s_q=np.zeros(N_l) # number of stopping veh on each lane
    s_er=np.zeros(N_l) # elapsed red time of each lane
    s_eg=np.zeros(N_l) # elapsed green time of each lane, size L
    perf_lane=np.zeros(N_l) # number of stopped vehicle through simulation period (T*L matrix)
    S=np.append(s_q,s_er) # state (N_l*2): [# stopped veh of eah lane, red elapsed sec for each lane]
    Snew=copy.copy(S)
    Q=np.zeros(N_p+1)
    Qnew=copy.copy(Q)
    A=0 # action
    Anew=A # new action
    
    # Assign initial values
    Plist=[] # list of phases storing y_set and all red, will pop through time if the list is not empty
    t=0 # step
    g_elps=0 # start decision after min green is over
    upd_dur=0 # update duration (time interval between actions)
    R=0 # total delay
    D=0 # departure veh
    curPhase=0 # current phase
    lastPhase=0
    
    # Open log files
    fout = open('log-W.txt', 'w')   
    fsar_out = open('log-SAR.txt', 'w')   
    # Start SUMO
    if _c.b_train:
#         traci.start(["sumo", "-c", mapName+".sumo.cfg"])
         traci.start(["sumo-gui", "-c", mapName+".sumo.cfg","--random"])
    else:
         traci.start(["sumo-gui", "-c", mapName+".sumo.cfg","--random"])
         
    # Start simulation         
    while t<_c.T:
         upd_dur+=1
         # get current simulation time
         ct=traci.simulation.getCurrentTime()/1000
         ######## Update state
         s_q,s_er,s_eg=updateS(s_q,s_er,s_eg) # since Snew=s_q, the func updates both
         R=R+calOBJ(laneSet,s_q,s_er)
         
         # still in phase transition(lost time), pop up a plan index each time
         if len(Plist): 
             traci.trafficlights.setRedYellowGreenState(TCid,Plist.pop(0))
             
         ######## take action and update weight
         if t%_c.updt_freq==0 and g_elps>=minG[curPhase]+ls_y+ls_r:
             ######## Update State
             Snew=np.append(s_q,s_er) # new state
             R=R/upd_dur # average stopping veh + penalty
             print "\n\nt = ",ct,"\nG_elps = ",g_elps,"\nA = ",A,"\nR = ",R
             
             ######## Choose an action
             Qnew,Anew,e=choseAction()
             print "Anew (e = ",e,") = ",Anew
             curPhase,Plist,g_elps=updatePhase(curPhase,g_elps)    
             # output S,A,R etc
             fsar_out.write(str(t) + ";" + str(S.tolist())[1:-1]+";"+str(Q)[1:-1]+";"+str(A)+";"+str(R)+"\n")
             ######## Update W
             if _c.b_train:
                 Wold=np.c_[Wc,We] # store old Ws
                 Wc,We=updateW()
                 dW=np.c_[Wc,We] - Wold
                 print "dW norm = ", np.linalg.norm(dW)
             S=copy.copy(Snew)
             Q=copy.copy(Qnew)
             A=Anew
             lastPhase=curPhase
             upd_dur=0
             R=0
             D=0
         perf_lane=np.vstack((perf_lane,s_q))
         traci.simulationStep()
         g_elps+=1
         t+=1
         
         # Output stuffs every since a while
         if t % 1800 ==0:
             fout.write("\nt = "+str(t)+"\nWe = \n"+str(We)+"\n")
             fout.write("\nWc = \n"+str(Wc)+"\n\n")
             
    if _c.b_outrlt and _c.b_train:
        Wc.dump('Training_rslt_Wc')
        We.dump('Training_rslt_We')
    if _c.b_outPerf:
        np.savetxt("sim_perf.csv", perf_lane,fmt='%03d', delimiter=",")
    stop = timeit.default_timer()
    print "computing time = ", stop-start
    traci.close()
    fout.close()
