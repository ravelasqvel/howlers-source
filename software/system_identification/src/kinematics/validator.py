"""
Kinematic validation tools for omnidirectional robots.

Provides tools to validate and visualize robot kinematics before parameter estimation.
"""

import numpy as np
import matplotlib.pyplot as plt


class KinematicValidator:
    """
    Tool to validate and visualize robot kinematics before parameter estimation.
    
    This class provides static methods for:
    - Computing kinematic transformation matrix
    - Visualizing wheel geometry
    - Testing kinematic consistency and controllability
    """
    
    @staticmethod
    def compute_H_matrix(wheel_angles, d, r):
        """
        Compute kinematic matrix from wheel geometry.
        
        For standard omni wheels (rollers perpendicular to wheel axis):
        H[i,:] = (1/r) * [-sin(θᵢ), cos(θᵢ), d]
        
        Parameters
        ----------
        wheel_angles : array (3,)
            Angular position of each wheel in DEGREES from robot's X-axis
        d : float
            Distance from robot center to wheel (m)
        r : float
            Wheel radius (m)
        
        Returns
        -------
        H : array (3, 3)
            Kinematic matrix: H @ [vx, vy, omega]^T = [w1, w2, w3]^T
        """
        theta = np.deg2rad(wheel_angles)
        
        H = np.zeros((3, 3))
        for i in range(3):
            H[i, 0] = -np.sin(theta[i])  # vx contribution
            H[i, 1] =  np.cos(theta[i])  # vy contribution
            H[i, 2] = -d                  # omega contribution (negative for CCW positive)
        
        H /= r
        return H
    
    @staticmethod
    def visualize_wheel_geometry(wheel_angles, d):
        """
        Plot robot geometry to visually verify wheel positions.
        
        Parameters
        ----------
        wheel_angles : array (3,)
            Wheel angular positions in degrees
        d : float
            Distance from center to wheels (m)
        
        Returns
        -------
        fig : matplotlib.figure.Figure
            Figure handle
        """
        fig, ax = plt.subplots(figsize=(9, 9))
        
        # Draw robot body (circle)
        circle = plt.Circle((0, 0), d, fill=False, color='black', linewidth=2.5)
        ax.add_patch(circle)
        
        # Draw coordinate frame
        arrow_len = d * 1.3
        ax.arrow(0, 0, arrow_len, 0, head_width=0.025, head_length=0.04, 
                fc='red', ec='red', linewidth=2.5, label='X-axis', zorder=10)
        ax.arrow(0, 0, 0, arrow_len, head_width=0.025, head_length=0.04, 
                fc='green', ec='green', linewidth=2.5, label='Y-axis', zorder=10)
        
        # Draw wheels
        theta = np.deg2rad(wheel_angles)
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c']  # Blue, Orange, Green
        wheel_labels = [f'w{i+1} ({int(wheel_angles[i])}°)' for i in range(3)]
        
        for i, (angle, color, label) in enumerate(zip(theta, colors, wheel_labels)):
            x = d * np.cos(angle)
            y = d * np.sin(angle)
            
            # Wheel position
            ax.plot(x, y, 'o', color=color, markersize=18, label=label, zorder=5)
            
            # Wheel orientation (tangent to circle = perpendicular to radius)
            tangent_angle = angle + np.pi/2
            dx = 0.08 * np.cos(tangent_angle)
            dy = 0.08 * np.sin(tangent_angle)
            ax.arrow(x-dx/2, y-dy/2, dx, dy, head_width=0.02, head_length=0.03, 
                    fc=color, ec=color, linewidth=2, alpha=0.7, zorder=5)
            
            # Radial line from center to wheel
            ax.plot([0, x], [0, y], '--', color=color, alpha=0.3, linewidth=1.5)
            
            # Angle label
            label_radius = d * 1.4
            label_x = label_radius * np.cos(angle)
            label_y = label_radius * np.sin(angle)
            ax.text(label_x, label_y, f'{wheel_angles[i]:.0f}°', 
                   fontsize=13, ha='center', va='center', 
                   bbox=dict(boxstyle='round,pad=0.4', facecolor='white', alpha=0.8))
        
        # Center point
        ax.plot(0, 0, 'ko', markersize=10, zorder=10)
        
        ax.set_xlim(-d*1.7, d*1.7)
        ax.set_ylim(-d*1.7, d*1.7)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper left', fontsize=11)
        ax.set_title('Robot Wheel Geometry\n(Looking from above)', 
                    fontsize=14, fontweight='bold')
        ax.set_xlabel('X [m]', fontsize=12)
        ax.set_ylabel('Y [m]', fontsize=12)
        
        plt.tight_layout()
        return fig
    
    @staticmethod
    def test_kinematic_consistency(H, verbose=True):
        """
        Test kinematic matrix for physical consistency.
        
        Tests performed:
        1. Pure X translation
        2. Pure Y translation
        3. Pure rotation (should be symmetric for 3-wheel config)
        4. Matrix rank = 3 (full controllability)
        5. Reconstructability: H @ H_pinv ≈ I
        
        Parameters
        ----------
        H : array (3, 3)
            Kinematic matrix to test
        verbose : bool
            Whether to print detailed results
        
        Returns
        -------
        results : dict
            Dictionary with test results
        """
        if verbose:
            print("=" * 70)
            print("KINEMATIC CONSISTENCY TESTS")
            print("=" * 70)
        
        results = {}
        
        # Test 1: Forward motion (X direction)
        v_forward = np.array([1.0, 0.0, 0.0])
        w_forward = H @ v_forward
        if verbose:
            print(f"\n1. Forward motion (vx = 1.0 m/s, X-direction):")
            print(f"   w1 = {w_forward[0]:+.4f} rad/s")
            print(f"   w2 = {w_forward[1]:+.4f} rad/s")
            print(f"   w3 = {w_forward[2]:+.4f} rad/s")
        results['forward'] = w_forward
        
        # Test 2: Sideways motion (Y direction)
        v_sideways = np.array([0.0, 1.0, 0.0])
        w_sideways = H @ v_sideways
        if verbose:
            print(f"\n2. Sideways motion (vy = 1.0 m/s, Y-direction):")
            print(f"   w1 = {w_sideways[0]:+.4f} rad/s")
            print(f"   w2 = {w_sideways[1]:+.4f} rad/s")
            print(f"   w3 = {w_sideways[2]:+.4f} rad/s")
        results['sideways'] = w_sideways
        
        # Test 3: Pure rotation
        v_rotation = np.array([0.0, 0.0, 1.0])
        w_rotation = H @ v_rotation
        if verbose:
            print(f"\n3. Pure rotation (omega = 1.0 rad/s, CCW):")
            print(f"   w1 = {w_rotation[0]:+.4f} rad/s")
            print(f"   w2 = {w_rotation[1]:+.4f} rad/s")
            print(f"   w3 = {w_rotation[2]:+.4f} rad/s")
            print(f"   Average magnitude: {np.mean(np.abs(w_rotation)):.4f} rad/s")
            
            # Check if all wheels contribute equally to rotation
            mag_std = np.std(np.abs(w_rotation))
            print(f"   Std of magnitudes: {mag_std:.6f}")
            if mag_std < 0.01:
                print(f"   ✓ All wheels contribute equally (symmetric)")
            else:
                print(f"   ⚠ Asymmetric contribution (check geometry)")
        results['rotation'] = w_rotation
        
        # Test 4: Matrix rank
        rank = np.linalg.matrix_rank(H)
        if verbose:
            print(f"\n4. Matrix rank: {rank}/3")
            if rank == 3:
                print(f"   ✓ Full rank → Robot is fully controllable")
            else:
                print(f"   ✗ Rank deficient → Robot has mobility constraints")
        results['rank'] = rank
        
        # Test 5: Reconstructability
        H_pinv = np.linalg.pinv(H)
        reconstruction_product = H @ H_pinv
        reconstruction_error = np.linalg.norm(reconstruction_product - np.eye(3), 'fro')
        if verbose:
            print(f"\n5. Reconstruction test (H @ H_pinv ≈ I):")
            print(f"   Frobenius norm error: {reconstruction_error:.2e}")
            if reconstruction_error < 1e-10:
                print(f"   ✓ Excellent reconstructability")
            elif reconstruction_error < 1e-6:
                print(f"   ✓ Good reconstructability")
            else:
                print(f"   ⚠ Poor reconstructability (numerical issues?)")
        results['reconstruction_error'] = reconstruction_error
        
        # Overall assessment
        all_pass = (rank == 3 and reconstruction_error < 1e-6)
        if verbose:
            print("\n" + "=" * 70)
            if all_pass:
                print("✓ ALL TESTS PASSED - Kinematics are correct")
            else:
                print("⚠ ISSUES DETECTED - Review wheel configuration")
            print("=" * 70 + "\n")
        
        results['all_pass'] = all_pass
        return results
