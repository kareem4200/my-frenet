import torch
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D
from model import CVAE, cvae_loss_function
import time

# neural network parameters
batch_size = 256
h_Q_dim = 512
h_P_dim = 512
z_dim = 3  # latent dimension
X_dim = 6  # input dimension (state)
c_dim = 133  # conditioning dimension (occ 121, init 6, goal 6)

gridSize = 11
num_viz = 3000    # 3000 samples to visualize

# Load the occupancy grid data and test conditions
occGridSamples = np.load('CVAE/data/occGridSamples.npy')

c_test = np.load('CVAE/data/c_test.npy')
c_gapsInitGoal = np.load('CVAE/data/c_gapsInitGoal.npy')

numTest = c_test.shape[0]
vizIdx = np.random.randint(0, numTest)
c_sample_seed = torch.tensor(c_test[vizIdx], dtype=torch.float32)
c_sample = c_sample_seed.unsqueeze(0).repeat(num_viz, 1)

c_viz = c_gapsInitGoal[vizIdx, :]

# Load the model
model = CVAE(X_dim=X_dim, c_dim=c_dim, z_dim=z_dim, h_Q_dim=h_Q_dim, h_P_dim=h_P_dim)
model.load_state_dict(torch.load('CVAE/model_weights/cvae_model.pth', weights_only=True))

# one inference step to generate 3000 samples
s_time = time.time()
with torch.inference_mode():
    z = torch.randn(num_viz, z_dim)
    y_pred = model.decode(z, c_sample)
    
print(f"Inference time for {num_viz} samples: {time.time() - s_time:.2f} seconds")

# Visualize the samples and the occupancy grid
fig1 = plt.figure(figsize=(10, 6), dpi=80)
ax1 = fig1.add_subplot(111, aspect='equal')

plt.scatter(y_pred[:,0], y_pred[:,1], color="green", s=70, alpha=0.1)

dw = 0.1
dimW = 3
gap1 = c_viz[0:3]
gap2 = c_viz[3:6]
gap3 = c_viz[6:9]
init = c_viz[9:15]
goal = c_viz[15:21]

obs1 = [0, gap1[1]-dw, -0.5,             gap1[0], gap1[1], 1.5]
obs2 = [gap2[0]-dw, 0, -0.5,             gap2[0], gap2[1], 1.5]
obs3 = [gap2[0]-dw, gap2[1]+dw, -0.5,    gap2[0], 1, 1.5]
obs4 = [gap1[0]+dw, gap1[1]-dw, -0.5,    gap3[0], gap1[1], 1.5]
obs5 = [gap3[0]+dw, gap1[1]-dw, -0.5,    1, gap1[1], 1.5]
obsBounds = [-0.1, -0.1, -0.5, 0, 1.1, 1.5,
            -0.1, -0.1, -0.5, 1.1, 0, 1.5,
            -0.1, 1, -0.5, 1.1, 1.1, 1.5,
            1, -0.1, -0.5, 1.1, 1.1, 1.5,]

obs = np.concatenate((obs1, obs2, obs3, obs4, obs5, obsBounds), axis=0)
for i in range(0, int(obs.shape[0]/(2*dimW))):
    ax1.add_patch(
    patches.Rectangle(
        (obs[i*2*dimW], obs[i*2*dimW+1]),   # (x,y)
        obs[i*2*dimW+dimW] - obs[i*2*dimW],          # width
        obs[i*2*dimW+dimW+1] - obs[i*2*dimW+1],          # height
        alpha=0.6
    ))
    
for i in range(0,gridSize*gridSize): # plot occupancy grid
    cIdx = i + 2*X_dim
    if c_sample_seed[cIdx] == 0:
        plt.scatter(occGridSamples[i,0], occGridSamples[i,1], color="red", s=50, alpha=0.7)
    else:
        plt.scatter(occGridSamples[i,0], occGridSamples[i,1], color="green", s=50, alpha=0.7)

plt.scatter(init[0], init[1], color="red", s=250, edgecolors='black') # init
plt.scatter(goal[0], goal[1], color="blue", s=250, edgecolors='black') # goal

plt.show()

plt.figure(figsize=(10,6), dpi=80)
viz1 = 1
viz2 = 4
plt.scatter(y_pred[:,viz1],y_pred[:,viz2], color="green", s=70, alpha=0.1)
plt.scatter(c_viz[viz1+9],c_viz[viz2+9], color="red", s=250, edgecolors='black') # init
plt.scatter(c_viz[viz1+9+X_dim],c_viz[viz2+9+X_dim], color="blue", s=500, edgecolors='black') # goal
plt.show()