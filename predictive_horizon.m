function [X_des, Y_des, theta_des] = predictive_horizon(X0, Ad, Bd, Wd, U0, label, x_r, y_r, theta_r, label_r, N)
% Predictive horizon block
%
% Inputs:
%   X0          : current state [X2; Y2; psi2; gamma]
%   Ad, Bd, Wd  : discrete-time affine model
%   U0          : nominal steering input
%   label       : current path segment label
%   x_r         : path x coordinates
%   y_r         : path y coordinates
%   theta_r     : path heading
%   label_r     : path labels
%
% Outputs:
%   X_des       : desired x positions over horizon
%   Y_des       : desired y positions over horizon
%   theta_des   : desired heading over horizon

    % Initialize predicted state
    Xk = X0;

    % Preallocate outputs
    X_des     = zeros(N,1);
    Y_des     = zeros(N,1);
    theta_des = zeros(N,1);

    % If the requested segment label does not exist, return zeros
    if ~any(label_r == label)
        return
    end

    % Build desired reference trajectory over the horizon
    for k = 1:N

        % Predict one step ahead using the current discrete model
        Xk = Ad * Xk + Bd * U0 + Wd;

        % Find closest point on the active path segment
        [theta_ref, ~, S_ref, ~] = distance_calc( ...
            Xk(1), Xk(2), label, x_r, y_r, theta_r, label_r);

        % Store desired reference values
        X_des(k)     = S_ref(1);
        Y_des(k)     = S_ref(2);
        theta_des(k) = theta_ref;
    end
end