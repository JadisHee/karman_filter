clear;
close all;
clc;
data = xlsread('data_0328_2');

% 读取数据
load('acc_.mat');
load('z.mat')

u = acc_(1,:);
t = (data(1:301,1))'/1000;

z = data(1:301,11)/1000000;

% x = data(1:301,11)/1000000;

v_var = 1;

% z = zeros(1,length(t));
% for i = 1:1:length(z)
%     
%     v_noise = 0 + v_var*randn();
%     z(i) = x(i) + v_noise;
% end

X = [0;0];

P = [0.0001,0;0,0.0001];
Q = [0.001,0;0,0.001];
H = [1,0];
R = 1;

X_record = zeros(1,length(t));

delta_t = zeros(1,length(t));
for i = 1:1:length(delta_t)
    if i == 1
        delta_t = 0;
    else
        delta_t = t(i) - t(i-1);
    end
    A = [1,delta_t;0,1];
    B = [0.5 * delta_t^2;delta_t];
    
    X_predict = A * X + u(i).*B;
    P_predict = A * P * A' + Q;
    
    K = P_predict * H' / (H * P_predict * H' + R);
    X = X_predict + K * (z(i) - H * X_predict);
    P = (eye(2) - K*H) * P_predict;
    X_record(i) = X(1,1);
    
end







figure()

plot(t,z)
hold on 
plot(t,X_record)
legend('测量位移','预测位移')

