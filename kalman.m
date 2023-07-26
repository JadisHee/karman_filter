% kalman filter
function X_hat = kalman(Z, u, dt, Q, R)
    X = 0;
    A = 1;
    B = dt;
    H = 1;
    P = 1;
    X_ = zeros(length(Z),1);
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
        
        X_(i) = X;
    end
    X_hat = X_;
end