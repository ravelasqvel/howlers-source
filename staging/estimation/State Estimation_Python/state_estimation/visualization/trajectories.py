"""
Trajectory and state visualization functions.

Provides functions for plotting robot trajectories, state estimates,
and comparing multiple filters.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle


def plot_trajectory(states, ground_truth=None, title="Robot Trajectory", 
                   figsize=(10, 8), save_path=None, show=True):
    """
    Plot 2D trajectory of robot.
    
    Parameters
    ----------
    states : np.ndarray or dict
        If array: states (N, dim_x) where first two columns are x, y
        If dict: dictionary with 'x' and 'y' keys
    ground_truth : np.ndarray or dict, optional
        Ground truth trajectory in same format as states
    title : str, optional
        Plot title
    figsize : tuple, optional
        Figure size (width, height)
    save_path : str, optional
        Path to save figure. If None, figure is not saved.
    show : bool, optional
        Whether to display the plot
        
    Returns
    -------
    fig, ax
        Matplotlib figure and axes
    """
    fig, ax = plt.subplots(figsize=figsize)
    
    # Extract x, y coordinates
    if isinstance(states, dict):
        x_est, y_est = states['x'], states['y']
    else:
        x_est, y_est = states[:, 0], states[:, 1]
    
    # Plot estimate
    ax.plot(x_est, y_est, 'b-', linewidth=2, label='Estimate', alpha=0.8)
    ax.plot(x_est[0], y_est[0], 'go', markersize=10, label='Start')
    ax.plot(x_est[-1], y_est[-1], 'r^', markersize=10, label='End')
    
    # Plot ground truth if available
    if ground_truth is not None:
        if isinstance(ground_truth, dict):
            x_true, y_true = ground_truth['x'], ground_truth['y']
        else:
            x_true, y_true = ground_truth[:, 0], ground_truth[:, 1]
        
        ax.plot(x_true, y_true, 'k--', linewidth=1.5, 
                label='Ground Truth', alpha=0.6)
    
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


def plot_states(time, states, ground_truth=None, state_names=None,
               title="State Estimates", figsize=(12, 8), save_path=None, show=True):
    """
    Plot all state variables over time.
    
    Parameters
    ----------
    time : np.ndarray
        Time vector (N,)
    states : np.ndarray
        State estimates (N, dim_x)
    ground_truth : np.ndarray, optional
        True states (N, dim_x)
    state_names : list of str, optional
        Names for each state dimension
    title : str, optional
        Main title for figure
    figsize : tuple, optional
        Figure size
    save_path : str, optional
        Path to save figure
    show : bool, optional
        Whether to display the plot
        
    Returns
    -------
    fig, axes
        Matplotlib figure and axes array
    """
    dim_x = states.shape[1]
    
    if state_names is None:
        state_names = [f'State {i+1}' for i in range(dim_x)]
    
    # Create subplots
    n_rows = (dim_x + 1) // 2
    fig, axes = plt.subplots(n_rows, 2, figsize=figsize)
    axes = axes.flatten()
    
    for i in range(dim_x):
        ax = axes[i]
        
        # Plot estimate
        ax.plot(time, states[:, i], 'b-', linewidth=2, label='Estimate', alpha=0.8)
        
        # Plot ground truth if available
        if ground_truth is not None:
            ax.plot(time, ground_truth[:, i], 'k--', linewidth=1.5, 
                   label='Ground Truth', alpha=0.6)
        
        ax.set_xlabel('Time (s)', fontsize=10)
        ax.set_ylabel(state_names[i], fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
    
    # Remove extra subplots if dim_x is odd
    if dim_x % 2 == 1:
        fig.delaxes(axes[-1])
    
    fig.suptitle(title, fontsize=14)
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    
    if show:
        plt.show()
    
    return fig, axes


def plot_comparison(time, estimates_dict, ground_truth=None, 
                   title="Filter Comparison", figsize=(16, 12), 
                   save_path=None, show=True):
    """
    Compare multiple filter estimates with 4 subplots:
    1. 2D trajectory (x-y plane)
    2. Orientation over time
    3. Position error over time
    4. Orientation error over time
    
    Parameters
    ----------
    time : np.ndarray
        Time vector (N,)
    estimates_dict : dict
        Dictionary mapping filter names to state estimates (N, dim_x)
        e.g., {'EKF': ekf_states, 'UKF': ukf_states, 'PF': pf_states}
        States assumed to be [x, y, theta, vx, vy, omega]
    ground_truth : np.ndarray, optional
        True state trajectory (N, dim_x)
    title : str, optional
        Main figure title
    figsize : tuple, optional
        Figure size (width, height)
    save_path : str, optional
        Path to save figure
    show : bool, optional
        Whether to display the plot
        
    Returns
    -------
    fig, axes
        Matplotlib figure and axes array
    """
    fig, axes = plt.subplots(2, 2, figsize=figsize)
    fig.suptitle(title, fontsize=16, fontweight='bold')
    
    # Color cycle for different filters
    colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple']
    
    # ========== Subplot 1: 2D Trajectory (x-y plane) ==========
    ax1 = axes[0, 0]
    
    # Plot ground truth first
    if ground_truth is not None:
        x_true, y_true = ground_truth[:, 0], ground_truth[:, 1]
        ax1.plot(x_true, y_true, 'k--', linewidth=2.5, label='Ground Truth', alpha=0.7, zorder=1)
        ax1.plot(x_true[0], y_true[0], 'go', markersize=12, label='Start', zorder=3)
        ax1.plot(x_true[-1], y_true[-1], 'r^', markersize=12, label='End', zorder=3)
    
    # Plot each filter's trajectory
    for idx, (filter_name, states) in enumerate(estimates_dict.items()):
        x_est, y_est = states[:, 0], states[:, 1]
        ax1.plot(x_est, y_est, linewidth=2, label=filter_name, 
                color=colors[idx % len(colors)], alpha=0.8, zorder=2)
    
    ax1.set_xlabel('X Position (m)', fontsize=11)
    ax1.set_ylabel('Y Position (m)', fontsize=11)
    ax1.set_title('2D Trajectory', fontsize=12, fontweight='bold')
    ax1.legend(fontsize=9, loc='best')
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # ========== Subplot 2: Orientation over Time ==========
    ax2 = axes[0, 1]
    
    # Plot ground truth
    if ground_truth is not None:
        theta_true = ground_truth[:, 2]
        ax2.plot(time, theta_true, 'k--', linewidth=2.5, label='Ground Truth', alpha=0.7)
    
    # Plot each filter's orientation
    for idx, (filter_name, states) in enumerate(estimates_dict.items()):
        theta_est = states[:, 2]
        ax2.plot(time, theta_est, linewidth=2, label=filter_name, 
                color=colors[idx % len(colors)], alpha=0.8)
    
    ax2.set_xlabel('Time (s)', fontsize=11)
    ax2.set_ylabel('Orientation (rad)', fontsize=11)
    ax2.set_title('Orientation vs Time', fontsize=12, fontweight='bold')
    ax2.legend(fontsize=9, loc='best')
    ax2.grid(True, alpha=0.3)
    
    # ========== Subplot 3: Position Error over Time ==========
    ax3 = axes[1, 0]
    
    if ground_truth is not None:
        x_true, y_true = ground_truth[:, 0], ground_truth[:, 1]
        
        # Calculate position error for each filter
        for idx, (filter_name, states) in enumerate(estimates_dict.items()):
            x_est, y_est = states[:, 0], states[:, 1]
            position_error = np.sqrt((x_est - x_true)**2 + (y_est - y_true)**2)
            ax3.plot(time, position_error, linewidth=2, label=filter_name, 
                    color=colors[idx % len(colors)], alpha=0.8)
    
    ax3.set_xlabel('Time (s)', fontsize=11)
    ax3.set_ylabel('Position Error (m)', fontsize=11)
    ax3.set_title('Position Error vs Time', fontsize=12, fontweight='bold')
    ax3.legend(fontsize=9, loc='best')
    ax3.grid(True, alpha=0.3)
    
    # ========== Subplot 4: Orientation Error over Time ==========
    ax4 = axes[1, 1]
    
    if ground_truth is not None:
        theta_true = ground_truth[:, 2]
        
        # Calculate orientation error for each filter (wrapped to [-pi, pi])
        for idx, (filter_name, states) in enumerate(estimates_dict.items()):
            theta_est = states[:, 2]
            # Wrap angle difference to [-pi, pi]
            theta_error = np.arctan2(np.sin(theta_est - theta_true), 
                                     np.cos(theta_est - theta_true))
            theta_error_abs = np.abs(theta_error)
            ax4.plot(time, theta_error_abs, linewidth=2, label=filter_name, 
                    color=colors[idx % len(colors)], alpha=0.8)
    
    ax4.set_xlabel('Time (s)', fontsize=11)
    ax4.set_ylabel('Orientation Error (rad)', fontsize=11)
    ax4.set_title('Orientation Error vs Time', fontsize=12, fontweight='bold')
    ax4.legend(fontsize=9, loc='best')
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"  Saved: {save_path}")
    
    if show:
        plt.show()
    
    return fig, axes


def plot_trajectory_with_particles(states, particles, weights, ground_truth=None,
                                   n_particles_to_plot=100, title="Particle Filter Trajectory",
                                   figsize=(10, 8), save_path=None, show=True):
    """
    Plot trajectory with particle cloud for Particle Filter.
    
    Parameters
    ----------
    states : np.ndarray
        State estimates (N, dim_x)
    particles : np.ndarray
        Particles at final time step (N_particles, dim_x)
    weights : np.ndarray
        Particle weights (N_particles,)
    ground_truth : np.ndarray, optional
        True trajectory
    n_particles_to_plot : int, optional
        Number of particles to visualize
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
           label='PF Estimate', alpha=0.8)
    
    if ground_truth is not None:
        ax.plot(ground_truth[:, 0], ground_truth[:, 1], 'k--', 
               linewidth=1.5, label='Ground Truth', alpha=0.6)
    
    # Plot particle cloud at final position
    # Sample particles based on weights
    indices = np.random.choice(len(particles), 
                              size=min(n_particles_to_plot, len(particles)),
                              p=weights, replace=False)
    
    for idx in indices:
        ax.plot(particles[idx, 0], particles[idx, 1], 'r.', 
               markersize=3, alpha=0.3)
    
    ax.plot([], [], 'r.', markersize=5, label='Particles', alpha=0.5)
    
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



def plot_trajectory_states(time, states, title="Trajectory Analysis", 
                           save_path=None, show=True):
    """
    Plot all 6 states of a robot trajectory.
    
    Generates a 2x3 subplot figure showing:
    - XY trajectory (2D path)
    - X, Y positions over time
    - Orientation (psi) over time
    - Body velocities (vx_b, vy_b) over time
    - Angular velocity (omega) over time
    
    Parameters
    ----------
    time : np.ndarray
        Time vector (N,)
    states : np.ndarray
        State trajectory (N, 6) where columns are [x, y, psi, vx_b, vy_b, omega]
    title : str, optional
        Main title for the figure
    save_path : str, optional
        Path to save the figure. If None, figure is not saved.
    show : bool, optional
        Whether to display the plot (default: True)
    
    Returns
    -------
    fig, axes
        Matplotlib figure and axes array
    """
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    fig.suptitle(title, fontsize=16, fontweight='bold')
    
    # Subplot 1: XY Trajectory (2D path)
    ax = axes[0, 0]
    ax.plot(states[:, 0], states[:, 1], 'b-', linewidth=2, label='Path')
    ax.scatter(states[0, 0], states[0, 1], c='green', s=150, marker='o', 
              label='Start', zorder=5, edgecolors='black', linewidths=1.5)
    ax.scatter(states[-1, 0], states[-1, 1], c='red', s=150, marker='^', 
              label='End', zorder=5, edgecolors='black', linewidths=1.5)
    ax.set_xlabel('X Position (m)', fontsize=11)
    ax.set_ylabel('Y Position (m)', fontsize=11)
    ax.set_title('2D Trajectory', fontsize=12, fontweight='bold')
    ax.axis('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9)
    
    # Subplot 2: X Position vs Time
    ax = axes[0, 1]
    ax.plot(time, states[:, 0], 'b-', linewidth=1.5)
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('X Position (m)', fontsize=11)
    ax.set_title('X Position vs Time', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    
    # Subplot 3: Y Position vs Time
    ax = axes[0, 2]
    ax.plot(time, states[:, 1], 'g-', linewidth=1.5)
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('Y Position (m)', fontsize=11)
    ax.set_title('Y Position vs Time', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    
    # Subplot 4: Orientation vs Time
    ax = axes[1, 0]
    ax.plot(time, states[:, 2], 'r-', linewidth=1.5)
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('Orientation (rad)', fontsize=11)
    ax.set_title('Orientation (ψ) vs Time', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    # Subplot 5: Body Frame Velocities
    ax = axes[1, 1]
    ax.plot(time, states[:, 3], 'c-', linewidth=1.5, label='vx_body')
    ax.plot(time, states[:, 4], 'm-', linewidth=1.5, label='vy_body')
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('Velocity (m/s)', fontsize=11)
    ax.set_title('Body Frame Velocities', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9)
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    # Subplot 6: Angular Velocity
    ax = axes[1, 2]
    ax.plot(time, states[:, 5], 'purple', linewidth=1.5, label='ω')
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('Angular Velocity (rad/s)', fontsize=11)
    ax.set_title('Angular Velocity vs Time', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9)
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"  Saved trajectory plot: {save_path}")
    
    if show:
        plt.show()
    else:
        plt.close(fig)
    
    return fig, axes
