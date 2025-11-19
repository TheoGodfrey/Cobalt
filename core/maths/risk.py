import numpy as np

class EntropicRisk:
    """
    Implements the Entropic Risk Measure:
    Psi_theta(C) = (1/theta) * ln( E[exp(theta * C)] )
    """
    def __init__(self, theta=0.5):
        """
        Args:
            theta: Risk sensitivity parameter.
                   theta -> 0 : Risk Neutral (Mean)
                   theta > 0  : Risk Averse (Penalizes variance)
        """
        self.theta = theta

    def compute_cost(self, nominal_cost, cost_variance=0.0):
        """
        Computes the risk-adjusted cost.
        
        If we assume the cost distribution is Gaussian N(mu, sigma^2),
        the entropic risk has a closed form solution:
        Psi = mu + (theta * sigma^2) / 2
        
        Args:
            nominal_cost (mu): The expected cost (mean).
            cost_variance (sigma^2): The variance of the cost (due to wind uncertainty).
        """
        if abs(self.theta) < 1e-6:
            # Limit as theta -> 0 is just the mean
            return nominal_cost
            
        # Closed form for Gaussian approximation (computationally cheap)
        return nominal_cost + (self.theta * cost_variance) / 2.0

    def compute_from_samples(self, cost_samples):
        """
        Computes risk using Monte Carlo samples of the cost.
        Use this if the cost distribution is non-Gaussian.
        """
        if abs(self.theta) < 1e-6:
            return np.mean(cost_samples)
            
        # Log-Sum-Exp trick for numerical stability
        # Psi = (1/theta) * ln( sum(exp(theta * c)) / N )
        costs = np.array(cost_samples)
        max_c = np.max(costs)
        
        sum_exp = np.sum(np.exp(self.theta * (costs - max_c)))
        log_term = np.log(sum_exp / len(costs))
        
        return max_c + (1.0 / self.theta) * log_term