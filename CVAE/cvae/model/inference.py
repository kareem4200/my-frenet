import torch
import numpy as np
from cvae.model.model import CVAE, cvae_loss_function
import time
import pandas as pd

# neural network parameters
batch_size = 128
h_Q_dim = 512
h_P_dim = 512
z_dim = 32  # latent dimension

# import data
X_test = pd.read_csv('cvae/data/x_test.csv') 
c_test = pd.read_csv('cvae/data/c_test.csv')

X_test = X_test.to_numpy()
c_test = c_test.to_numpy()

X_test_tensor = torch.tensor(X_test, dtype=torch.float32)
c_test_tensor = torch.tensor(c_test, dtype=torch.float32)

test_dataset = torch.utils.data.TensorDataset(X_test_tensor, c_test_tensor)
test_dataloader = torch.utils.data.DataLoader(test_dataset, batch_size=batch_size, shuffle=True)

X_dim = X_test_tensor.shape[1]
c_dim = c_test_tensor.shape[1]

# Load the model
model = CVAE(X_dim=X_dim, c_dim=c_dim, z_dim=z_dim, h_Q_dim=h_Q_dim, h_P_dim=h_P_dim)
model.load_state_dict(torch.load('cvae/model/weights/cvae_model_lr_0.005_batch_128_epochs_100_zdim_32_kl_beta_1.0.pth'))
model.eval()

# one inference step to generate 3000 samples
s_time = time.time()
with torch.inference_mode():
    loss = 0
    for batch in test_dataloader:
        x, c = batch
        y_pred, mu, logvar = model(x, c)
        # print(y_pred, x)
        batch_loss = cvae_loss_function(y_pred, x, mu, logvar)
        
        loss += batch_loss.item()
        print(f"batch reconstruction loss: {batch_loss.item():.4f}")
    print(f"Total reconstruction loss: {loss:.4f}")