# -*- coding: utf-8 -*-
"""
User-defined parameters for the TD algorithm
"""
# Training or simulation
b_train = True
# File i/o settings
b_outlog=False
b_outPerf=True # output performance through simulation(delay)
b_outrlt=True # output the finaled training result (weight w)

# Learning parameters (parameters for updating w)
beta=0.9 # discounting rate 
alpha_max=0.0000001 # learning rate 
alpha_min=0.00000000000001
alpha_maxite=1000000.0
updt_freq=2 # freq of updating signal phase (and value approximator)
T=9999999999999999 # total steps

# OBJ parameters
tol_er = 15 # the time length that penalty doesnt occur
pen_rate = 0.5 # slope (assume penaly incresing linearly with time)

# Updating mathod for weight w
#updateScheme="SARSA"
updateScheme="Q"

# e-greedy parameters
# decent e rate, e = max (min_p,max_p-t/max_ite)
# thus e = min_p when t =(maxp-minp)*maxite
e_max_p=0.8 # initial e
e_min_p=0.0 # minimum e
e_max_ite=1000000.0

