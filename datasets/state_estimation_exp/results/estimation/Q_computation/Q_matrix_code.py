"""Recommended Q matrix from model identification"""
import numpy as np

# Process noise covariance matrix Q
# Computed from experimental data
# Experiment: 2
# Acceleration variance: sigma_ax = 2.3064, sigma_ay = 2.1359
# Time step: dt = 0.0100 s

Q = np.diag([
    1.228769e-08,  # x
    1.241803e-08,  # y
    1.000000e-12,  # phi
    4.914677e-04,  # vx
    4.966813e-04,  # vy
    1.000000e-12,  # omega
])
