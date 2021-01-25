# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""
import pathlib
import os
import numpy as np
import cv2
import matplotlib.pyplot as plt
import yaml
import pandas as pd 

#path="/home/jacobi/.ros/narrow_pass/1611415310.475126/dof3" #ellisse
tests=os.listdir("/home/jacobi/.ros/narrow_pass/");
results=[]

plotfig=False
for dof in range(2,11):
    
    for test in tests:
        path="/home/jacobi/.ros/narrow_pass/"+test+"/dof"+str(dof) #tubo
        if not os.path.exists(path):
            continue
        
        with open(path+'/description.yaml') as f:
            description = yaml.load(f, Loader=yaml.FullLoader)
        
        if plotfig:
            if (description["tube_sampler"]):
                if description['forgetting_factor'] != 0.999:
                    continue
                if description['tube_radius'] != 0.01:
                    continue
            
        global_minimum= description['global_minimum']
        local_minimum= description['local_minimum']
        if (description["tube_sampler"]):
            description['gain']=description['reward']/(1-description['forgetting_factor'])
        else:
            description['gain']=float("nan")
        
        files = os.listdir(path)
        trial_files = [i for i in files if i.endswith('.dirrt')]
        
        iter1perc=np.array([])
        for file in trial_files:
            xbash = np.fromfile(path+"/"+file, dtype='float')
            if len(xbash)==0:
                continue
            
            if len(xbash)%5!=0:
                continue
            data=xbash.reshape(int(len(xbash)/5.0),5)
            iterations=data[:,1]
            cost=data[:,4]
            idx1p=np.where(cost>1.01*global_minimum)[-1]
            if len(idx1p)>0:
                iter1perc = np.append(iter1perc, np.where(cost>1.01*global_minimum)[-1][-1])
            else:
                if (cost[-1]<=1.01*global_minimum):
                    iter1perc=np.append(iter1perc, iterations[-1])
                else:
                    iter1perc=np.append(iter1perc, float("nan"))

        if len(iter1perc)<=1:
            continue
        description["percentile90"]=np.percentile(iter1perc,90)
        description["percentile50"]=np.percentile(iter1perc,50)
        description["dof"]=dof
        results.append(description)
        if plotfig:
            #plt.xscale("log")
            plt.yscale("log")
            
            left, right = plt.xlim()
            plt.xlim(1,10000)
            #plt.plot([left,right],[global_minimum,global_minimum],'--k')
            plt.plot([left,right],[local_minimum/global_minimum-1,local_minimum/global_minimum-1],'--r')
            plt.grid()
            
            if (description["tube_sampler"]):
                plt.title("dof "+str(dof)+"\n"
                          "forgetting_factor "+str(description['forgetting_factor'])+"\n"+
                          "reward "+str(description['reward'])+"\n"+
                          "tube_radius "+str(description['tube_radius']))
            else:
                plt.title("dof "+str(dof)+"\n"
                          "Informed RRT*")
                
            plt.show()

df = pd.DataFrame(results)
df.to_excel("~/.ros/narrow_pass/results.xlsx")