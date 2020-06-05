"""Sample Buffers

This script provides sample buffers for Rl alogorithms.

Class:
    * PPOBuffer - sample buffer (list) for the PPO algorithm
    * PPOBuffer2 - sample buffer (dequeu) for the PPO algorithm
    * A3CMemory - sample buffer for n-step A3C
"""
import torch
from collections import deque


class PPOBuffer(object):
    """
    Replay Buffer to save samples for PPO
    and diagnostic training data
    """

    def __init__(self, batch_size, norm_adv=True):
        # Samples
        self.obs = [[], [], []]
        self.actions = []
        self.logps = []
        self.values = []
        self.returns = []
        self.advantages = []
        # Diagnostics
        self.episode_rewards = []
        self.episode_lengths = []

        self.norm_adv = norm_adv
        self.batch_size = batch_size
        self.buffer_size = 0

    def append(self, obs_t, action_t, logp_t, value_t, return_t, advantage_t):
        """Adds a sample to the buffer"""
        self.obs[0].append(obs_t[0])
        self.obs[1].append(obs_t[1])
        self.obs[2].append(obs_t[2])
        self.actions.append(action_t)
        self.logps.append(logp_t)
        self.values.append(value_t)
        self.returns.append(return_t)
        self.advantages.append(advantage_t)
        self.buffer_size += 1

    def eject(self):
        """Prepares and returns the collected batch"""
        # Convert batch to tensors
        (obs, actions, logps, values, returns, advantages) = self.batch_to_tensor()
        # Normalize advantages
        if self.norm_adv:
            advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-5)

        return obs, actions, logps, values, returns, advantages

    def batch_to_tensor(self):
        """Transforms batch to torch.Tensors"""
        # Convert arrays/vectors to torch.Tensors
        xV = torch.Tensor(self.obs[0]).float()
        xE = torch.Tensor(self.obs[1]).float()
        xO = torch.Tensor(self.obs[2]).float()

        # For LSTM
        # xO = [torch.Tensor(o).float() for o in self.obs[2]]

        obs = [xV, xE, xO]
        actions = torch.Tensor(self.actions).float()
        logps = torch.Tensor(self.logps).float()
        values = torch.Tensor(self.values).float()
        returns = torch.Tensor(self.returns).float()
        advantages = torch.Tensor(self.advantages).float()

        return obs, actions, logps, values, returns, advantages

    def flush(self):
        """Clears the buffer"""
        self.obs = [[], [], []]
        self.actions = []
        self.logps = []
        self.values = []
        self.returns = []
        self.advantages = []
        self.episode_rewards = []
        self.episode_lengths = []
        self.buffer_size = 0

    def __len__(self):
        """Returns the current batch size"""
        return self.buffer_size


class PPOBuffer2(object):
    """
    Replay Buffer to save samples for PPO
    and diagnostic training data
    """

    def __init__(self, batch_size, norm_adv=False):
        # Samples
        self.obs = [deque(maxlen=batch_size),
                    deque(maxlen=batch_size),
                    deque(maxlen=batch_size)]
        self.actions = deque(maxlen=batch_size)
        self.logps = deque(maxlen=batch_size)
        self.values = deque(maxlen=batch_size)
        self.returns = deque(maxlen=batch_size)
        self.advantages = deque(maxlen=batch_size)
        # Diagnostics
        self.episode_rewards = deque(maxlen=batch_size)
        self.episode_lengths = deque(maxlen=batch_size)

        self.norm_adv = norm_adv
        self.batch_size = batch_size
        self.buffer_size = 0

    def append(self, obs_t, action_t, logp_t, value_t, return_t, advantage_t):
        """Adds a sample to the buffer"""
        if self.buffer_size < self.batch_size:
            self.buffer_size += 1

        self.obs[0].append(obs_t[0])
        self.obs[1].append(obs_t[1])
        self.obs[2].append(obs_t[2])
        self.actions.append(action_t)
        self.logps.append(logp_t)
        self.values.append(value_t)
        self.returns.append(return_t)
        self.advantages.append(advantage_t)

    def eject(self):
        """Prepares and returns the collected batch"""
        # Convert batch to tensors
        (obs, actions, logps, values, returns, advantages) = self.batch_to_tensor()
        # Normalize advantages
        if self.norm_adv:
            advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-5)

        self.buffer_size = 0

        return obs, actions, logps, values, returns, advantages

    def batch_to_tensor(self):
        """Transforms batch to torch.Tensors"""
        # Convert arrays/vectors to torch.Tensors
        xV = torch.Tensor(self.obs[0]).float()
        xE = torch.Tensor(self.obs[1]).float()
        xO = torch.Tensor(self.obs[2]).float()

        # For LSTM
        # xO = [torch.Tensor(o).float() for o in self.obs[2]]

        obs = [xV, xE, xO]
        actions = torch.Tensor(self.actions).float()
        logps = torch.Tensor(self.logps).float()
        values = torch.Tensor(self.values).float()
        returns = torch.Tensor(self.returns).float()
        advantages = torch.Tensor(self.advantages).float()

        return obs, actions, logps, values, returns, advantages

    def __len__(self):
        """Returns the current batch size"""
        return self.buffer_size


class A3CMemory(object):
    """
    Memory to save n-steps
    """

    def __init__(self):
        self.log_probs = []
        self.entropies = []
        self.values = []
        self.rewards = []

    def store(self, log_prob, entropy, value, reward):
        self.log_probs.append(log_prob)
        self.entropies.append(entropy)
        self.values.append(value)
        self.rewards.append(reward)

    def get_history(self):
        return iter(zip(self.log_probs[::-1],
                        self.entropies[::-1],
                        self.values[::-1],
                        self.rewards[::-1]))

    def clear(self):
        self.log_probs = []
        self.entropies = []
        self.values = []
        self.rewards = []
