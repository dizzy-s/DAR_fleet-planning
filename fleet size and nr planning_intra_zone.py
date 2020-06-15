# -*- coding: utf-8 -*-
"""
Created on Mon Jun  1 15:41:14 2020

@author: shenshiyu
"""

import numpy as np
import matplotlib.pyplot as plt

#Global variable
X = 5

K = 0.57 #distance factor
v = 20 #velocity of vehicle
C = 3 #vehicle capacity
m_max = 15 #max vehicle

#Input
lamda_0 = 5 #demand density
R = 25 #area of the region
l = 4 #average travel distance for passengers in the region
tw_max = 30/60 #max passenger waiting time
te_max = 30/60 #max passenger detour time

"""
#Derivation
nw = ( (m*v) / (lamda * K * R**(3/2)) - nr**(-1/2) )**(-2)
tw_ave = nw/(lamda*R)
tr_ave = m*nr/(lamda*R)
ts_ave = l/v
te_ave = tr_ave - ts_ave
lamda = (1 - tw_ave/tw_max)*(1 - te_ave/te_max)*lamda_0

#Objective
obj = lamda/m #increase lamda, decrease m

#Constraint
cons1 = nr - ( (lamda * K * R**(3/2)) / (m*v) )**2 #should >= 0 
cons2 = te_ave #should >= 0
cons3 = tw_max - tw_ave #should >= 0 
cons4 = te_max - te_ave #should >= 0
cons5 = C - nr #should >= 0
"""

#Calculation for optimal combination of m and nr
#initialize m and nr values
m_list = list(range(1, m_max+1, 1)) #generate possible number of vehicles
nr_list = np.arange(1, C+0.01, 0.01).tolist() #generate possible value of nr

#update derivations
def update_lamda(m, nr, lamda):
    
    nw, tw_ave, tr_ave, ts_ave, te_ave = 0, 0, 0, 0, 0       
    nw = ( (m*v) / (lamda * K * R**(3/2)) - nr**(-1/2) )**(-2)
    tw_ave = nw/(lamda*R)
    tr_ave = m*nr/(lamda*R)
    ts_ave = l/v
    te_ave = tr_ave - ts_ave 
#    lamda_new = (1 - tw_ave/tw_max)*(1 - te_ave/te_max)*lamda_0
    lamda_new = (1 - tw_ave/(tw_max**2))*(1 - te_ave/(te_max**2))*lamda_0

    
    return lamda_new, nw, tw_ave, tr_ave, ts_ave, te_ave

def find_lamda(m, nr):
    
    nw, tw_ave, tr_ave, ts_ave, te_ave = 0, 0, 0, 0, 0
    n = 1
    lamda_1 = update_lamda(m, nr, lamda_0)[0]
    lamda_list = [lamda_0, lamda_1]
    while n <= 10000:
        
        lamda_new, nw, tw_ave, tr_ave, ts_ave, te_ave = update_lamda(m, nr, lamda_list[n])       
        
        delta = lamda_new - lamda_list[n]
        
        if abs(delta) <= 0.0001:
            break
        
        else:
            lamda_new_1 = (1/X)*lamda_new + (1-1/X)*lamda_list[n]

        lamda_list.append(lamda_new_1)
        n += 1
            
    if abs(lamda_list[-1]-lamda_list[-2]) >= 0.001:
        lamda_list = [0]

    return lamda_list[-1], nw, tw_ave, tr_ave, ts_ave, te_ave
    
#check feasibility
def check_feas(m, nr, lamda, nw, tw_ave, tr_ave, ts_ave, te_ave):

    cons1 = nr - ( (lamda * K * R**(3/2)) / (m*v) )**2 #should >= 0 
    cons2 = te_ave #should >= 0
    cons3 = tw_max - tw_ave #should >= 0 
    cons4 = te_max - te_ave #should >= 0
    cons5 = C - nr #should >= 0
    
    check_list = np.array([cons1, cons2, cons3, cons4, cons5])
    #check if all constraints >= 0
    return (check_list >= 0).all() 

#calculate objective
obj_list = np.zeros((len(m_list),len(nr_list)))
lamda_list, tw_ave_list, tr_ave_list, te_ave_list = np.zeros((len(m_list),len(nr_list))), np.zeros((len(m_list),len(nr_list))), np.zeros((len(m_list),len(nr_list))), np.zeros((len(m_list),len(nr_list)))
for i in m_list:
    for j in nr_list:
        lamda, nw, tw_ave, tr_ave, ts_ave, te_ave = find_lamda(i, j)
        if check_feas(i, j, lamda, nw, tw_ave, tr_ave, ts_ave, te_ave) == True:
            obj_list[m_list.index(i),nr_list.index(j)] = lamda/i
            lamda_list[m_list.index(i),nr_list.index(j)] = lamda
            tw_ave_list[m_list.index(i),nr_list.index(j)] = tw_ave
            tr_ave_list[m_list.index(i),nr_list.index(j)] = tr_ave
            te_ave_list[m_list.index(i),nr_list.index(j)] = te_ave
"""
Global optimal solution
"""
#m_loc, nr_loc = np.where(obj_list == obj_list.max())
#m_loc = m_loc[0]
#nr_loc = nr_loc[0]

"""
Fixed fleet size solution
"""
m_loc = 11
obj_slice = obj_list[m_loc,:].tolist()
nr_loc = obj_slice.index(max(obj_slice))

"""
Fixed fleet size and occupancy rate
"""
#m_loc = 11
#nr_loc = 2

solution_list = []
solution = (m_list[m_loc], nr_list[nr_loc], lamda_list[m_loc, nr_loc], tw_ave_list[m_loc, nr_loc], tr_ave_list[m_loc, nr_loc], te_ave_list[m_loc, nr_loc])
solution_list.append(solution)

#Print&Plot result
print('{}: {} {}'.format('Fleet size',solution_list[0][0], 'veh' ))
print('{}: {:.4} {}'.format('Occupancy rate',solution_list[0][1], 'pax/veh' ))
print('{}: {:.4} {}'.format('Service Fulfillment Rate',100*solution_list[0][2]/lamda_0, '%' ))
print('{}: {:.4} {}'.format('Average Waiting Time',solution_list[0][3]*60, 'min' ))
print('{}: {:.4} {}'.format('Average Riding Time',solution_list[0][4]*60, 'min' ))
print('{}: {:.4} {}'.format('Average Detour Time',solution_list[0][5]*60, 'min' ))

plt.axis([1,C,0,1])
for i in range(len(nr_list)):
    if lamda_list[m_loc,i] != 0:
        plt.scatter(nr_list[i], lamda_list[m_loc,i]/lamda_0, s=50, c='g', marker = '+', alpha = 0.5)
plt.scatter(nr_list[nr_loc], lamda_list[m_loc,nr_loc]/lamda_0, s=50, c='r', marker = '+', alpha = 1)
