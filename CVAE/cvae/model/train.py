import torch
import numpy as np
from cvae.model.model import CVAE, cvae_loss_function
from tqdm import tqdm
import pandas as pd
from torch.utils.tensorboard import SummaryWriter


device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Using device: {device}")

# neural network parameters
batch_size = 128
# h_Q_dim = 128
# h_P_dim = 128
z_dim = 32  # latent dimension
# X_dim = 6  # input dimension (state)
# c_dim = 133  # conditioning dimension (occ 121, init 6, goal 6)

lr = 0.002
num_epochs = 100
# kl_beta = 1e-0  # KL divergence weight
kl_beta = 1.0

# import data
X_train = pd.read_csv('cvae/data/x_train.csv') 
c_train = pd.read_csv('cvae/data/c_train.csv')

X_val = pd.read_csv('cvae/data/x_validation.csv')
c_val = pd.read_csv('cvae/data/c_validation.csv')

X_train = X_train.to_numpy()
c_train = c_train.to_numpy()

X_val = X_val.to_numpy()
c_val = c_val.to_numpy()

X_train_tensor = torch.tensor(X_train, dtype=torch.float32)
c_train_tensor = torch.tensor(c_train, dtype=torch.float32)

X_val_tensor = torch.tensor(X_val, dtype=torch.float32)
c_val_tensor = torch.tensor(c_val, dtype=torch.float32)

X_dim = X_train_tensor.shape[1]
c_dim = c_train_tensor.shape[1]

train_dataset = torch.utils.data.TensorDataset(X_train_tensor, c_train_tensor)
train_dataloader = torch.utils.data.DataLoader(train_dataset, batch_size=batch_size, shuffle=True)

val_dataset = torch.utils.data.TensorDataset(X_val_tensor, c_val_tensor)
val_dataloader = torch.utils.data.DataLoader(val_dataset, batch_size=batch_size, shuffle=True)

# model
model = CVAE(X_dim, c_dim, z_dim).to(device)
optimizer = torch.optim.Adam(model.parameters(), lr=lr)
# lr_scheduler = torch.optim.lr_scheduler.ExponentialLR(optimizer, gamma=0.995)
# lr_scheduler = torch.optim.lr_scheduler.StepLR(optimizer,  step_size=100, gamma=0.8)
lr_scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', factor=0.5, patience=5)

writer = SummaryWriter(
      log_dir=f'cvae/model/runs/lr_{lr}_batch_{batch_size}_epochs_{num_epochs}_zdim_{z_dim}_kl_beta_{kl_beta}'
      )

for epoch in range(num_epochs):
      epoch_loss = 0.0
      
      # ----- Training Step -----
      model.train()
      for batch in tqdm(train_dataloader, desc="Training Progress"):
            x, c = batch
            x, c = x.to(device), c.to(device)
            
            y_pred, mu, logvar = model(x, c)
            # kl_beta = min(0.2, 0.2 * epoch / num_epochs)
            loss = cvae_loss_function(y_pred, x, mu, logvar, kl_beta=kl_beta)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            
            epoch_loss += loss.item()
            
      avg_epoch_loss = epoch_loss / len(train_dataloader)
            
      # ----- Validation Step -----
      model.eval()
      val_loss = 0
      with torch.no_grad():
            for batch in val_dataloader:
                  x, c = batch
                  x, c = x.to(device), c.to(device)
                  
                  output, mu, logvar = model(x, c)
                  loss = cvae_loss_function(output, x, mu, logvar, kl_beta=kl_beta)
                  val_loss += loss.item()
            
      avg_val_loss = val_loss / len(val_dataloader)
      # lr_scheduler.step()
      
      writer.add_scalar('Loss/Train', avg_epoch_loss, epoch)
      writer.add_scalar('Loss/Val', avg_val_loss, epoch)
            
      print(f"Epoch {epoch+1}/{num_epochs} | Train Loss: {avg_epoch_loss:.4f} | Val Loss: {avg_val_loss:.4f}")
      
torch.save(
      model.state_dict(), 
      f'cvae/model/weights/cvae_model_lr_{lr}_batch_{batch_size}_epochs_{num_epochs}_zdim_{z_dim}_kl_beta_{kl_beta}.pth'
      )
