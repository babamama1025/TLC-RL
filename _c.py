# -*- coding: utf-8 -*-
"""
User-defined parameters for the TD algorithm
"""
# Training or simulation
b_train = True

# Learning parameters
alpha,beta=[0.0001,0.9] # parameters for updating w
updt_freq=20 # freq of updating signal phase (and value approximator)
T=150000 # total steps

# Updating mathod for weight w
#updateScheme="SARSA"
updateScheme="Q"

# e-greedy parameters
# decent e rate, e = max (min_p,max_p-t/max_ite)
# thus e = min_p when t =(maxp-minp)*maxite
e_max_p=0.8 # initial e
e_min_p=0.05 # minimum e
e_max_ite=100000.0

# File i/o settings
b_outlog=False
b_outrlt=True # output the finaled training result (weight w)