"""
Covariance and uncertainty visualization.

Functions for plotting uncertainty ellipses and confidence regions.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from scipy.stats import chi2


def plot_covariance_ellipse(mean, cov, n_std=3.0, ax=None, **kwargs):
    """
    Plot covariance ellipse for 2D distribution.
    
    Parameters
    ----------
    mean : array-like
        Mean of distribution [x, y]
    cov : np.ndarray
        2x2 covariance matrix
    n_std : float, optional
        Number of standard deviations for ellipse (default: 3-sigma)
    ax : matplotlib.axes.Axes, optional
        Axes to plot on. If None, creates new figure.
    **kwargs : dict
        Additional arguments passed to Ellipse patch
        (e.g., facecolor, edgecolor, alpha, linewidth)
        
    Returns
    -------
    matplotlib.patches.Ellipse
        The ellipse patch object
    """
    if ax is None:
        ax = plt.gca()
    
    mean = np.asarray(mean)
    cov = np.asarray(cov)
    
    # Compute eigenvalues and eigenvectors
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    
    # Angle of ellipse (first eigenvector)
    angle = np.degrees(np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0]))
    
    # Width and height (scaled by eigenvalues and n_std)
    width, height = 2 * n_std * np.sqrt(eigenvalues)
    
    # Create ellipse
    ellipse = Ellipse(xy=mean, width=width, height=height, angle=angle, **kwargs)
    
    ax.add_patch(ellipse)
    
    return ellipse


def plot_uncertainty(states, covariances, ground_truth=None, 
                    n_std=3.0, n_ellipses=10,
                    title="State Estimation with Uncertainty",
                    figsize=(10, 8), save_path=None, show=True):
    """
    Plot trajectory with uncertainty ellipses.
    
    Parameters
    ----------
    states : np.ndarray
        State estimates (N, dim_x)
    covariances : np.ndarray
        State covariances (N, dim_x, dim_x)
    ground_truth : np.ndarray, optional
        True states (N, dim_x)
    n_std : float, optional
        Number of standard deviations for ellipses
    n_ellipses : int, optional
        Number of ellipses to plot along trajectory
    title : str, optional
        Plot title
    figsize : tuple, optional
        Figure size
    save_path : str, optional
        Path to save figure
    show : bool, optional
        Whether to display the plot
        
    Returns
    -------
    fig, ax
        Matplotlib figure and axes
    """
    fig, ax = plt.subplots(figsize=figsize)
    
    # Plot trajectory
    ax.plot(states[:, 0], states[:, 1], 'b-', linewidth=2, 
           label='Estimate', alpha=0.8)
    
    if ground_truth is not None:
        ax.plot(ground_truth[:, 0], ground_truth[:, 1], 'k--', 
               linewidth=1.5, label='Ground Truth', alpha=0.6)
    
    # Plot uncertainty ellipses at regular intervals
    N = len(states)
    indices = np.linspace(0, N-1, n_ellipses, dtype=int)
    
    for idx in indices:
        mean = states[idx, :2]  # x, y position
        cov = covariances[idx, :2, :2]  # 2x2 position covariance
        
        plot_covariance_ellipse(mean, cov, n_std=n_std, ax=ax,
                              facecolor='lightblue', edgecolor='blue',
                              alpha=0.3, linewidth=1)
    
    ax.set_xlabel('X Position (m)', fontsize=12)
    ax.set_ylabel('Y Position (m)', fontsize=12)
    ax.set_title(title, fontsize=14)
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    
    if show:
        plt.show()
    
    return fig, ax


def plot_nees(nees_values, dim_x, confidence=0.95,
             title="NEES Consistency Test",
             figsize=(12, 6), save_path=None, show=True):
    """
    Plot Normalized Estimation Error Squared (NEES) with confidence bounds.
    
    Parameters
    ----------
    nees_values : np.ndarray
        NEES values over time (N,)
    dim_x : int
        Dimension of state vector
    confidence : float, optional
        Confidence level for bounds (default: 0.95)
    title : str, optional
        Plot title
    figsize : tuple, optional
        Figure size
    save_path : str, optional
        Path to save figure
    show : bool, optional
        Whether to display the plot
        
    Returns
    -------
    fig, ax
        Matplotlib figure and axes
    """
    fig, ax = plt.subplots(figsize=figsize)
    
    time = np.arange(len(nees_values))
    
    # Plot NEES
    ax.plot(time, nees_values, 'b-', linewidth=1.5, alpha=0.7, label='NEES')
    
    # Expected value (mean of chi-squared distribution)
    expected = dim_x
    ax.axhline(expected, color='k', linestyle='--', linewidth=2, 
              label=f'Expected ({dim_x})')
    
    # Confidence bounds
    alpha = 1 - confidence
    lower_bound = chi2.ppf(alpha/2, dim_x)
    upper_bound = chi2.ppf(1 - alpha/2, dim_x)
    
    ax.axhline(lower_bound, color='r', linestyle=':', linewidth=1.5,
              label=f'{confidence*100:.0f}% Bounds')
    ax.axhline(upper_bound, color='r', linestyle=':', linewidth=1.5)
    ax.fill_between(time, lower_bound, upper_bound, alpha=0.1, color='red')
    
    ax.set_xlabel('Time Step', fontsize=12)
    ax.set_ylabel('NEES', fontsize=12)
    ax.set_title(title, fontsize=14)
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    
    if show:
        plt.show()
    
    return fig, ax


def plot_nis(nis_values, dim_z, confidence=0.95,
            title="NIS Consistency Test",
            figsize=(12, 6), save_path=None, show=True):
    """
    Plot Normalized Innovation Squared (NIS) with confidence bounds.
    
    Parameters
    ----------
    nis_values : np.ndarray
        NIS values over time (N,)
    dim_z : int
        Dimension of measurement vector
    confidence : float, optional
        Confidence level for bounds
    title : str, optional
        Plot title
    figsize : tuple, optional
        Figure size
    save_path : str, optional
        Path to save figure
    show : bool, optional
        Whether to display the plot
        
    Returns
    -------
    fig, ax
        Matplotlib figure and axes
    """
    fig, ax = plt.subplots(figsize=figsize)
    
    time = np.arange(len(nis_values))
    
    # Plot NIS
    ax.plot(time, nis_values, 'b-', linewidth=1.5, alpha=0.7, label='NIS')
    
    # Expected value
    expected = dim_z
    ax.axhline(expected, color='k', linestyle='--', linewidth=2,
              label=f'Expected ({dim_z})')
    
    # Confidence bounds
    alpha = 1 - confidence
    lower_bound = chi2.ppf(alpha/2, dim_z)
    upper_bound = chi2.ppf(1 - alpha/2, dim_z)
    
    ax.axhline(lower_bound, color='r', linestyle=':', linewidth=1.5,
              label=f'{confidence*100:.0f}% Bounds')
    ax.axhline(upper_bound, color='r', linestyle=':', linewidth=1.5)
    ax.fill_between(time, lower_bound, upper_bound, alpha=0.1, color='red')
    
    ax.set_xlabel('Time Step', fontsize=12)
    ax.set_ylabel('NIS', fontsize=12)
    ax.set_title(title, fontsize=14)
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    
    if show:
        plt.show()
    
    return fig, ax
