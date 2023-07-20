%% ͨ��imuԤ����ٶ�����̼Ƽ�������λ�ƽ���һά�������ں�
% ����imuԤ����ٶ���Ϊ���������룬����Ԥ������λ��
% ��̼Ƽ�������λ����Ϊ�۲���������У�����������ֵ

clear;
close all;
clc;

% ��ȡ�ɼ��������ĵ�
data = xlsread('./data/record_2.csv');

% �����������λs
dt = 0.016;

% imuԤ����ٶ�
velocity_imu = data(:,3);

% ��̼��ٶ�
velocity_odom = data(:,2);
% ����۲���������ͨ����̼��ٶȼ�����۲�λ��
Z = zeros(length(velocity_odom(:)),1);
for i = 2:1:length(Z)
    Z(i) = Z(i-1) + velocity_odom(i-1) * dt;
end

% ������
u = velocity_imu;
% ���岢��ʼ��״̬������Ԥ������
X = 0;
% ����״̬����
A = 1;
% ������ƾ���
B = dt;
% ����Ԥ�������������
Q = 0.001;

% ����۲�����״̬�����Ĺ�ϵ����
H = 1;
% ������������
R = 1;

% �������ʼ��Э�������
P = 1;

% ����Ԥ�������������ڴ���ÿ��Ԥ����ֵ
X_hat = zeros(length(velocity_odom(:)),1);
% ����
for i = 1:1:length(Z)
    % �������
    X_predict = A * X + B * u(i);
    % �������Э�������
    P_predict = A * P * A' + Q;
    
    % У������
    % ����������
    K = P_predict * H' / (H * P_predict * H' + R);
    % �������
    X = X_predict + K * (Z(i) - H * X_predict);
    % ����Э�������
    P = (1 - K * H) * P_predict;
    
    % ��ֵ
    X_hat(i) = X;
end

%% ��ͼ
figure(1)
plot(Z)
hold on
plot(X_hat)
legend('��ʽ��̼Ʋ���ֵ','����������ֵ')
