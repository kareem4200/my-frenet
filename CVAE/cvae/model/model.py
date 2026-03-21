import torch
import torch.nn as nn
import torch.nn.functional as F

class CVAE(nn.Module):
    def __init__(self, X_dim, c_dim, z_dim, h_Q_dim=512, h_P_dim=512):
        super(CVAE, self).__init__()
        
        # Encoder (Q network)
        self.fc_q1 = nn.Linear(X_dim + c_dim, h_Q_dim)
        self.fc_q2 = nn.Linear(h_Q_dim, h_Q_dim)
        self.fc_mu = nn.Linear(h_Q_dim, z_dim)
        self.fc_logvar = nn.Linear(h_Q_dim, z_dim)
        
        # Decoder (P network)
        self.fc_p1 = nn.Linear(z_dim + c_dim, h_P_dim)
        self.fc_p2 = nn.Linear(h_P_dim, h_P_dim)
        self.fc_out = nn.Linear(h_P_dim, X_dim)

    def encode(self, x, c):
        xc = torch.cat([x, c], dim=1)
        h = F.relu(self.fc_q1(xc))
        # h = F.dropout(h, p=0.5, training=self.training)
        h = F.relu(self.fc_q2(h))
        mu = self.fc_mu(h)
        logvar = self.fc_logvar(h)
        return mu, logvar

    def reparameterize(self, mu, logvar):
        std = torch.exp(0.5 * logvar)
        eps = torch.randn_like(std)
        return mu + eps * std

    def decode(self, z, c):
        zc = torch.cat([z, c], dim=1)
        h = F.relu(self.fc_p1(zc))
        # h = F.dropout(h, p=0.5)
        h = F.relu(self.fc_p2(h))
        return self.fc_out(h)

    def forward(self, x, c):
        mu, logvar = self.encode(x, c)
        z = self.reparameterize(mu, logvar)
        y = self.decode(z, c)
        return y, mu, logvar


def cvae_loss_function(y_pred, y_true, mu, logvar, kl_beta=1e-4):
    recon_loss = F.mse_loss(y_pred, y_true)
    # if weight is not None:
    #     recon_loss = recon_loss * weight
    kl_loss = -0.5 * torch.mean(1 + logvar - mu.pow(2) - logvar.exp(), dim=1) # normalized over latent dimension trick
    kl_loss = kl_loss.mean()
    # kl_loss = max(0.2, kl_loss)
    # print(f"KL Loss: {kl_loss}, Recon Loss: {recon_loss}")
    return recon_loss + kl_beta * kl_loss