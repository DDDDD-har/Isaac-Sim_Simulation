import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

def run_analysis():
    file_path = 'tracking_performance.csv'
    if not os.path.exists(file_path):
        print(f"Error: CSV file '{file_path}' not found.")
        return

    df = pd.read_csv(file_path)
    
    # Calculate Metrics
    rmse = np.sqrt(np.mean(df['err_y']**2))
    max_err = df['err_y'].abs().max()

    print(f"--- Tracking Analysis Results ---")
    print(f"RMSE (Root Mean Square Error): {rmse:.4f} m")
    print(f"Max Lateral Deviation: {max_err:.4f} m")

    plt.figure(figsize=(12, 10))

    # Subplot 1: Trajectory Tracking
    plt.subplot(2, 1, 1)
    plt.plot(df['ref_x'], df['ref_y'], 'r--', label='Reference Path', alpha=0.8)
    plt.plot(df['x'], df['y'], 'b-', label='Actual Trajectory', linewidth=2)
    plt.title('Vehicle Trajectory Tracking (Lateral PID)', fontsize=14)
    plt.xlabel('X Position [m]', fontsize=12)
    plt.ylabel('Y Position [m]', fontsize=12)
    plt.legend(loc='best')
    plt.axis('equal')
    plt.grid(True, linestyle=':', alpha=0.6)

    # Subplot 2: Cross-Track Error Over Time
    plt.subplot(2, 1, 2)
    plt.plot(df['err_y'], 'g-', label='Lateral Error (e_y)')
    plt.axhline(y=0, color='black', linewidth=1.2)
    plt.fill_between(range(len(df)), df['err_y'], 0, color='green', alpha=0.1)
    plt.title('Cross-Track Error Over Samples', fontsize=14)
    plt.xlabel('Sample Index', fontsize=12)
    plt.ylabel('Error [m]', fontsize=12)
    plt.legend(loc='best')
    plt.grid(True, linestyle=':', alpha=0.6)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    run_analysis()