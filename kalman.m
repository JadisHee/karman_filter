% kalman filter
function X_hat = kalman(Z, u, dt, Q, R)
    X = 0;
    A = 1;
    B = dt;
    H = 1;
    P = 1;
    X_ = zeros(length(Z),1);
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
        
        X_(i) = X;
    end
    X_hat = X_;
end