%% 通过imu预测的速度与里程计计算所得位移进行一维卡尔曼融合
% 其中imu预测的速度作为控制量输入，用于预测先验位移
% 里程计计算所得位移作为观测量，用于校正出后验估计值

clear;
close all;
clc;

% 读取采集的数据文档
data = xlsread('./data/record_2.csv');

% 采样间隔，单位s
dt = 0.016;

% imu预测的速度
velocity_imu = data(:,3);

% 里程计速度
velocity_odom = data(:,2);
% 定义观测量，并把通过里程计速度计算出观测位移
Z = zeros(length(velocity_odom(:)),1);
for i = 2:1:length(Z)
    Z(i) = Z(i-1) + velocity_odom(i-1) * dt;
end

% 控制量
u = velocity_imu;
% 定义并初始化状态变量（预测量）
X = 0;
% 定义状态矩阵
A = 1;
% 定义控制矩阵
B = dt;
% 定义预测过程噪声方差
Q = 0.001;

% 定义观测量与状态变量的关系矩阵
H = 1;
% 测量噪声方差
R = 1;

% 定义与初始化协方差矩阵
P = 1;

% 定义预测量向量，用于储存每次预测后的值
X_hat = zeros(length(velocity_odom(:)),1);
% 迭代
for i = 1:1:length(Z)
    % 先验估计
    X_predict = A * X + B * u(i);
    % 先验估计协方差矩阵
    P_predict = A * P * A' + Q;
    
    % 校正过程
    % 卡尔曼增益
    K = P_predict * H' / (H * P_predict * H' + R);
    % 后验估计
    X = X_predict + K * (Z(i) - H * X_predict);
    % 更新协方差矩阵
    P = (1 - K * H) * P_predict;
    
    % 赋值
    X_hat(i) = X;
end

%% 绘图
figure(1)
plot(Z)
hold on
plot(X_hat)
legend('轮式里程计测量值','卡尔曼估计值')
