# Workspace problem with several narrow gaps

import torch
import numpy as np
from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D
import os
import pandas as pd
from random import randint, random
import time


def isSampleFree(sample, obs):
    for o in range(0, int(obs.shape[0]/(2*dimW))):
        isFree = 0
        for d in range(0, sample.shape[0]):
            if (sample[d] < obs[2*dimW*o + d] or sample[d] > obs[2*dimW*o + d + dimW]):
                isFree = 1
                break
        if isFree == 0:
            return 0
    return 1


# problem dimensions
dim = 6

# 27 in total, 6 for the state and 21 for the conditions
dataElements = dim+3*3+2*dim # sample (6D), gap1 (2D, 1D orientation), gap2, gap3, init (6D), goal (6D)

z_dim = 3 # latent
X_dim = dim # samples
y_dim = dim # reconstruction of the original point
c_dim = dataElements - dim # dimension of conditioning variable

data_df = pd.read_csv(os.getcwd() + '/CVAE/data/narrowDataFile.txt')
data_df = data_df.drop(columns=data_df.columns[-1])  # Drop the last column

# convert the data to numpy array of float32
data_np = data_df.to_numpy()
data_np = data_np.astype(np.float32)

# split the inputs and conditions into test train (to be processed in the next step into an occupancy grid representation)
numEntries = data_np.shape[0]
ratioTestTrain = 0.8
numTrain = int(numEntries*ratioTestTrain)

X_train = data_np[0:numTrain, 0:dim] # state: x, y, z, xdot, ydot, zdot
c_train = data_np[0:numTrain, dim:dataElements] # conditions: gaps, init (6), goal (6)

X_test = data_np[numTrain:numEntries, 0:dim]
c_test = data_np[numTrain:numEntries, dim:dataElements]
numTest = X_test.shape[0]

gridSize = 11
dimW = 3
plotOn = False

# process data into occupancy grid
conditions = data_np[0:numEntries, dim:dataElements]
conditionsOcc = np.zeros([numEntries, gridSize*gridSize])
occGridSamples = np.zeros([gridSize*gridSize, 2]) # 121 x 2
gridPointsRange = np.linspace(0, 1, gridSize)

# Fill the occupancy grid samples with the grid points
idx = 0
for i in gridPointsRange:
    for j in gridPointsRange:
        occGridSamples[idx, 0] = i
        occGridSamples[idx, 1] = j
        idx += 1


start = time.time()
for j in range(0, numEntries, 1): # loop through all samples
    dw = 0.1
    dimW = 3
    gap1 = conditions[j, 0:3]
    gap2 = conditions[j, 3:6]
    gap3 = conditions[j, 6:9]
    init = conditions[j, 9:15]
    goal = conditions[j, 15:21]

    obs1 = [0, gap1[1] - dw, -0.5, gap1[0], gap1[1], 1.5] # 0 1 2 3 4 5
    obs2 = [gap2[0] - dw, 0, -0.5, gap2[0], gap2[1], 1.5] # 6 7 8 9 10 11
    obs3 = [gap2[0] - dw, gap2[1] + dw, -0.5, gap2[0], 1, 1.5] # 12 13 14 15 16 17
    obs4 = [gap1[0] + dw, gap1[1] - dw, -0.5, gap3[0], gap1[1], 1.5] # 18 19 20 21 22 23
    obs5 = [gap3[0] + dw, gap1[1] - dw, -0.5, 1, gap1[1], 1.5] # 24 25 26 27 28 29
    obs = np.concatenate((obs1, obs2, obs3, obs4, obs5), axis=0)
    #print('Obs: ', obs)
    
    if j % 5000 == 0:
        print('Iter: {}'.format(j))
        
    occGrid = np.zeros(gridSize*gridSize)
    for i in range(0,gridSize*gridSize):
        occGrid[i] = isSampleFree(occGridSamples[i, :], obs)
    conditionsOcc[j, :] = occGrid   # every sample has its own occupancy grid (how?)
    
    if plotOn:
        fig1 = plt.figure(figsize=(10,6), dpi=80)
        ax1 = fig1.add_subplot(111, aspect='equal')
        for i in range(0, int(obs.shape[0]/(2*dimW))): # plot obstacle patches
            ax1.add_patch(
            patches.Rectangle(
                (obs[i*2*dimW], obs[i*2*dimW+1]),   # (x,y)
                obs[i*2*dimW+dimW] - obs[i*2*dimW],          # width
                obs[i*2*dimW+dimW+1] - obs[i*2*dimW+1],          # height
                alpha=0.6
            ))
        for i in range(0,gridSize*gridSize): # plot occupancy grid
            if occGrid[i] == 0:
                plt.scatter(occGridSamples[i,0], occGridSamples[i,1], color="red", s=70, alpha=0.8)
            else:
                plt.scatter(occGridSamples[i,0], occGridSamples[i,1], color="green", s=70, alpha=0.8)
        plt.show()
end = time.time()
print('Time: ', end-start)

cs = np.concatenate((data_np[0:numEntries, dim+3*dimW:dataElements], conditionsOcc), axis=1) # occ, init, goal
c_dim = cs.shape[1]
c_gapsInitGoal = c_test
c_train = cs[0:numTrain, :]
c_test = cs[numTrain:numEntries, :]


np.save('CVAE/data/X_train.npy', X_train)
np.save('CVAE/data/c_train.npy', c_train)
np.save('CVAE/data/X_test.npy', X_test)
np.save('CVAE/data/c_test.npy', c_test)
np.save('CVAE/data/c_gapsInitGoal.npy', c_gapsInitGoal)
np.save('CVAE/data/occGridSamples.npy', occGridSamples)
