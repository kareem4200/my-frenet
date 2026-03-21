import torch
import numpy as np
from model import CVAE, cvae_loss_function
from tqdm import tqdm


device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Using device: {device}")

# neural network parameters
batch_size = 256
h_Q_dim = 512
h_P_dim = 512
z_dim = 3  # latent dimension
X_dim = 6  # input dimension (state)
c_dim = 133  # conditioning dimension (occ 121, init 6, goal 6)

lr = 1e-4
num_epochs = 5000
loss_weight = torch.tensor([[1, 1, 1, 0.5, 0.5, 0.5]], device=device, dtype=torch.float32)

# import data
X = np.load('CVAE/data/X_train.npy')
c = np.load('CVAE/data/c_train.npy')

X_tensor = torch.tensor(X, dtype=torch.float32)
c_tensor = torch.tensor(c, dtype=torch.float32)

dataset = torch.utils.data.TensorDataset(X_tensor, c_tensor)
dataloader = torch.utils.data.DataLoader(dataset, batch_size=batch_size, shuffle=True)

# model
model = CVAE(X_dim, c_dim, z_dim).to(device)
optimizer = torch.optim.Adam(model.parameters(), lr=lr)
# lr_scheduler = torch.optim.lr_scheduler.ExponentialLR(optimizer, gamma=0.995)
# lr_scheduler = torch.optim.lr_scheduler.StepLR(optimizer,  step_size=100, gamma=0.8)
# lr_scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', factor=0.5, patience=5, verbose=True)

for epoch in range(num_epochs):
      epoch_loss = 0.0
    
      for batch in tqdm(dataloader, desc="Training Progress"):
            x, c = batch
            x, c = x.to(device), c.to(device)
            
            y_pred, mu, logvar = model(x, c)
            loss = cvae_loss_function(y_pred, x, mu, logvar, weight=loss_weight)

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            
            epoch_loss += loss.item()
            
      # lr_scheduler.step()
            
      print(f"Epoch [{epoch+1}/{num_epochs}], Loss: {epoch_loss:.4f}")
      
torch.save(model.state_dict(), 'CVAE/model_weights/cvae_model.pth')
