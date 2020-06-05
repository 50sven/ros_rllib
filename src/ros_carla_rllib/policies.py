"""Policies

This script provides policies for RL algorithms.

Class:
    * Model - (arbitrary) neural network architecture
    * BetaActor - actor with beta policy
    * GaussianActor - actor with gaussian policy
    * Critic - state value function
    * ActorCritic - actor and critic combined
"""
import torch
import numpy as np
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import MultivariateNormal, Beta


class Model(nn.Module):

    def __init__(self, out_size):
        super(Model, self).__init__()

        # Visual
        self.conv1 = nn.Conv2d(3, 32, 8, 4)
        self.conv2 = nn.Conv2d(32, 64, 4, 2)
        self.conv3 = nn.Conv2d(64, 64, 3, 1)
        self.fcV1 = nn.Linear(12288, 512)

        # Numeric
        self.fcN1 = nn.Linear(18, 128)
        self.fcN2 = nn.Linear(128, 128)

        # Combined
        self.fcC1 = nn.Linear(512 + 128, 256)

        # Action
        self.fcOut = nn.Linear(256, out_size)

    def forward(self, obs):
        """ """
        xV, xE, _ = obs[0], obs[1], obs[2]

        # Visual
        xV = F.relu(self.conv1(xV))
        xV = F.relu(self.conv2(xV))
        xV = F.relu(self.conv3(xV))
        xV = torch.flatten(xV, 1)
        xV = F.relu(self.fcV1(xV))

        # Numeric
        xE = F.relu(self.fcN1(xE))
        xE = F.relu(self.fcN2(xE))

        # Combined
        xC = torch.cat([xE, xV], 1)
        xC = F.relu(self.fcC1(xC))

        # Output
        out = self.fcOut(xC)
        return out


class Actor(nn.Module):

    def get_dist(self, obs):
        raise NotImplementedError

    def get_logp(self, pi, action):
        raise NotImplementedError

    def forward(self, obs, action=None):
        pi = self.get_dist(obs)
        logp = None
        if action is not None:
            logp = self.get_logp(pi, action)
        return pi, logp

class BetaActor(Actor):

    def __init__(self, model):
        super(BetaActor, self).__init__()

        self.model = globals()[model](out_size=4)

    def get_dist(self, obs):
        concentration = self.model(obs)
        alpha = concentration[:, :2]
        beta = concentration[:, 2:]
        return Beta(alpha, beta)

    def get_logp(self, pi, action):
        return pi.log_prob(action).sum(1)


class GaussianActor(Actor):

    def __init__(self, model):
        super(GaussianActor, self).__init__()

        self.model = globals()[model](out_size=2)
        log_std = np.array([0.55, -0.35], dtype=np.float32)
        self.log_std = torch.nn.Parameter(torch.as_tensor(log_std))

    def get_dist(self, obs):
        mu = self.model(obs)
        std = torch.exp(self.log_std)
        return MultivariateNormal(mu, scale_tril=torch.diag_embed(std))

    def get_logp(self, pi, action):
        return pi.log_prob(action)


class Critic(nn.Module):

    def __init__(self, model):
        super(Critic, self).__init__()

        self.model = globals()[model](out_size=1)

    def forward(self, obs):
        """ """
        return self.model(obs)


class ActorCritic(object):

    def __init__(self, model, policy="gaussian", device="cpu"):

        policy = policy.capitalize() + "Actor"
        self.pi = globals()[policy](model).to(device)
        self.value_fn = Critic(model).to(device)

    def act(self, obs):
        """Returns (deterministic) action and state value"""
        with torch.no_grad():
            pi = self.pi.get_dist(obs)
            value = self.value_fn(obs)
        return pi.mean.numpy(), value.item()

    def sample(self, obs):
        """Returns sampled action, log probability and state-value"""
        with torch.no_grad():
            pi = self.pi.get_dist(obs)
            sample = pi.sample()
            logp = self.pi.get_logp(pi, sample)
            value = self.value_fn(obs)
        return sample.numpy(), logp.item(), value.item()
