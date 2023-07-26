import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# 读取csv文件
data = pd.read_csv('./data/record_2_no_title.csv').values

# Sampling interval, in seconds
dt = 0.016

# IMU predicted velocity
velocity_imu = data[:, 2]

displacement_imu = data[:, 4]

displacement_slam = data[:, 3]

# Odometry velocity
velocity_odom = data[:, 1]

# Define observation vector Z and compute observed displacement using odometry velocity
Z = np.zeros_like(velocity_odom)
for i in range(1, len(Z)):
    Z[i] = Z[i - 1] + velocity_odom[i - 1] * dt

# Control input
u = velocity_imu

# Initialize state variable (predicted value)
X = 0

# State matrix
A = 1

# Control matrix
B = dt

# Process noise variance
Q = 0.001

# Observation-state mapping matrix
H = 1

# Measurement noise variance
R = 1

# Initialize and define covariance matrix
P = 1

# Define vector to store predicted values at each iteration
X_hat = np.zeros_like(velocity_odom)

# Kalman filter loop
for i in range(len(Z)):
    # Prediction step
    X_predict = A * X + B * u[i]
    P_predict = A * P * A + Q

    # Correction step
    K = P_predict * H / (H * P_predict * H + R)
    X = X_predict + K * (Z[i] - H * X_predict)
    P = (1 - K * H) * P_predict

    # Store the updated value
    X_hat[i] = X

# Plotting the results
plt.figure(1)
plt.plot(Z, label='Odometry Measurement')
plt.plot(X_hat, label='Kalman Estimate')
plt.plot(displacement_imu, label='IMU Displacement')
plt.plot(displacement_slam, label='SLAM Displacement')
plt.legend()
plt.show()