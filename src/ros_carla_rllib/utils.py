"""Utilities

This script provides further functionalities.

Class:
    * RunningStatistics - tracks running mean/var statistics
"""
import torch
import numpy as np


class RunningStatistics(object):

    # https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Parallel_algorithm

    def __init__(self, epsilon=1e-4):
        self.Mean_ = 0.0
        self.Var_ = 1.0
        self.count = epsilon

    def mean(self):
        return self.Mean_

    def var(self):
        return self.Var_

    def std(self):
        return np.sqrt(self.Var_)

    def update(self, x):
        """ """
        if isinstance(x, torch.Tensor):
            batch_mean = torch.mean(x, axis=0).item()
            batch_var = torch.var(x, axis=0).item()
        else:
            batch_mean = np.mean(x, axis=0)
            batch_var = np.var(x, axis=0)
        batch_count = x.shape[0]
        self.Mean_, self.Var_, self.count = self.update_from_moments(self.Mean_, self.Var_, self.count,
                                                                     batch_mean, batch_var, batch_count)

    def update_from_moments(self, mean, var, count, batch_mean, batch_var, batch_count):
        """ """
        delta = batch_mean - mean
        tot_count = count + batch_count

        new_mean = mean + delta * batch_count / tot_count
        m_a = var * count
        m_b = batch_var * batch_count
        M2 = m_a + m_b + delta**2 * count * batch_count / tot_count
        new_var = M2 / tot_count
        new_count = tot_count

        return new_mean, new_var, new_count


if __name__ == "__main__":

    # Testing RunningStatistics()

    x = [torch.randn(64), torch.randn(128), torch.randn(256)]
    x_cat = torch.cat(x, axis=0)
    rms = RunningStatistics()
    for x_mb in x:
        rms.update(x_mb)

    ms1 = [x_cat.mean(0), x_cat.var(0), x_cat.std(0)]
    ms2 = [rms.mean(), rms.var(), rms.std()]
    print(ms1)
    print(ms2)

    print("-"*20)

    x = [np.random.randn(64), np.random.randn(128), np.random.randn(256)]
    x_cat = np.concatenate(x, axis=0)
    rms = RunningStatistics()
    for x_mb in x:
        rms.update(x_mb)

    ms1 = [x_cat.mean(0), x_cat.var(0), x_cat.std(0)]
    ms2 = [rms.mean(), rms.var(), rms.std()]
    print(ms1)
    print(ms2)
