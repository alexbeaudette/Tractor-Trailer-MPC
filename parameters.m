classdef parameters
    % Parameter config for baseline truck-trailer MPC

    properties (Constant)
        % Vehicle geometry
        L1 = 3.261;          % wheelbase of truck
        L1c = parameters.L1 - 3; % length between hitch point and rear axle of truck
        L2 = 10.0;           % length of trailer
        Llook = 2.0;         % lookahead distance for gamma_ref

        % Sampling time
        Ts = 0.01;

        % MPC dimensions
        nx = 4;   % states: [X2; Y2; psi2; gamma]
        nu = 1;   % steering input
        N  = 15;  % prediction horizon

        % Constraints
        umin = -pi/6;
        umax =  pi/6;

        % Steering rate constraint
        max_steer_rate = deg2rad(25); % rad/s
        delta_umin = -parameters.Ts * parameters.max_steer_rate;
        delta_umax =  parameters.Ts * parameters.max_steer_rate;

        % Cost weights
        % States: [X2; Y2; psi2; gamma]
        Q_fwd = diag([100, 100, 50, 50]);
        Q_rev = diag([100, 100, 50, 200]);
        R = 1;
        P = 50;
    end
end