"""
Performance metrics for evaluating state estimation quality.

Includes RMSE, MAE, NEES, and NIS for comprehensive filter evaluation.
"""

import numpy as np


def rmse(estimates, ground_truth, axis=0):
    """
    Root Mean Square Error.
    
    Parameters
    ----------
    estimates : np.ndarray
        Estimated states (N, dim) or (N,)
    ground_truth : np.ndarray
        True states (N, dim) or (N,)
    axis : int, optional
        Axis along which to compute RMSE
        
    Returns
    -------
    float or np.ndarray
        RMSE value(s)
    """
    estimates = np.asarray(estimates)
    ground_truth = np.asarray(ground_truth)
    
    squared_errors = (estimates - ground_truth) ** 2
    mean_squared_error = np.mean(squared_errors, axis=axis)
    
    return np.sqrt(mean_squared_error)


def mae(estimates, ground_truth, axis=0):
    """
    Mean Absolute Error.
    
    Parameters
    ----------
    estimates : np.ndarray
        Estimated states
    ground_truth : np.ndarray
        True states
    axis : int, optional
        Axis along which to compute MAE
        
    Returns
    -------
    float or np.ndarray
        MAE value(s)
    """
    estimates = np.asarray(estimates)
    ground_truth = np.asarray(ground_truth)
    
    absolute_errors = np.abs(estimates - ground_truth)
    
    return np.mean(absolute_errors, axis=axis)


def nees(estimates, ground_truth, covariances):
    """
    Normalized Estimation Error Squared (NEES).
    
    Measures consistency of the estimator. For a consistent filter,
    NEES should follow a chi-squared distribution with dim_x degrees of freedom.
    
    Parameters
    ----------
    estimates : np.ndarray
        Estimated states (N, dim_x)
    ground_truth : np.ndarray
        True states (N, dim_x)
    covariances : np.ndarray
        Estimation error covariances (N, dim_x, dim_x)
        
    Returns
    -------
    np.ndarray
        NEES values for each time step (N,)
        
    Notes
    -----
    For a consistent filter, the average NEES should be approximately dim_x.
    """
    N = len(estimates)
    nees_values = np.zeros(N)
    
    for i in range(N):
        error = estimates[i] - ground_truth[i]
        P_inv = np.linalg.inv(covariances[i])
        nees_values[i] = error.T @ P_inv @ error
    
    return nees_values


def nis(innovations, innovation_covariances):
    """
    Normalized Innovation Squared (NIS).
    
    Measures consistency of the measurement updates. For a consistent filter,
    NIS should follow a chi-squared distribution with dim_z degrees of freedom.
    
    Parameters
    ----------
    innovations : np.ndarray
        Innovation vectors (N, dim_z)
    innovation_covariances : np.ndarray
        Innovation covariances (N, dim_z, dim_z)
        
    Returns
    -------
    np.ndarray
        NIS values for each time step (N,)
        
    Notes
    -----
    For a consistent filter, the average NIS should be approximately dim_z.
    """
    N = len(innovations)
    nis_values = np.zeros(N)
    
    for i in range(N):
        y = innovations[i]
        S_inv = np.linalg.inv(innovation_covariances[i])
        nis_values[i] = y.T @ S_inv @ y
    
    return nis_values


def compute_all_metrics(estimates, ground_truth, covariances=None, 
                       innovations=None, innovation_covariances=None):
    """
    Compute all available metrics.
    
    Parameters
    ----------
    estimates : np.ndarray
        Estimated states (N, dim_x)
    ground_truth : np.ndarray
        True states (N, dim_x)
    covariances : np.ndarray, optional
        State covariances (N, dim_x, dim_x)
    innovations : np.ndarray, optional
        Innovation vectors (N, dim_z)
    innovation_covariances : np.ndarray, optional
        Innovation covariances (N, dim_z, dim_z)
        
    Returns
    -------
    dict
        Dictionary with computed metrics
    """
    metrics = {}
    
    # Basic metrics (always available)
    metrics['rmse'] = rmse(estimates, ground_truth, axis=0)
    metrics['mae'] = mae(estimates, ground_truth, axis=0)
    metrics['rmse_total'] = float(np.mean(rmse(estimates, ground_truth, axis=0)))
    metrics['mae_total'] = float(np.mean(mae(estimates, ground_truth, axis=0)))
    
    # Consistency metrics (require covariances)
    if covariances is not None:
        nees_vals = nees(estimates, ground_truth, covariances)
        metrics['nees'] = nees_vals
        metrics['nees_mean'] = float(np.mean(nees_vals))
        metrics['nees_std'] = float(np.std(nees_vals))
    
    if innovations is not None and innovation_covariances is not None:
        nis_vals = nis(innovations, innovation_covariances)
        metrics['nis'] = nis_vals
        metrics['nis_mean'] = float(np.mean(nis_vals))
        metrics['nis_std'] = float(np.std(nis_vals))
    
    return metrics


def print_metrics(metrics, filter_name="Filter"):
    """
    Print metrics in a formatted way.
    
    Parameters
    ----------
    metrics : dict
        Dictionary of metrics from compute_all_metrics
    filter_name : str, optional
        Name of the filter for display
    """
    print(f"\n{filter_name} Performance Metrics")
    print("=" * 50)
    
    if 'rmse' in metrics:
        print(f"RMSE per dimension: {metrics['rmse']}")
    if 'rmse_total' in metrics:
        print(f"Total RMSE: {metrics['rmse_total']:.6f}")
    
    if 'mae' in metrics:
        print(f"MAE per dimension: {metrics['mae']}")
    if 'mae_total' in metrics:
        print(f"Total MAE: {metrics['mae_total']:.6f}")
    
    if 'nees_mean' in metrics:
        print(f"NEES (mean ± std): {metrics['nees_mean']:.2f} ± {metrics['nees_std']:.2f}")
    
    if 'nis_mean' in metrics:
        print(f"NIS (mean ± std): {metrics['nis_mean']:.2f} ± {metrics['nis_std']:.2f}")
    
    print("=" * 50)
