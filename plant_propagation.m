function X_next = plant_propagation(X, delta, ux)
% Propagate truck-trailer kinematic model one step.
%
% Inputs:
%   X     : current state [x2; y2; psi2; gamma]
%   delta : front steering angle [rad]
%   ux    : longitudinal speed at tractor rear axle [m/s]
%
% Outputs:
%   X_next : next state [x2; y2; psi2; gamma]
%
% State definitions:
%   x2, y2 : trailer rear axle position
%   psi2   : trailer heading
%   gamma  : articulation angle = psi1 - psi2
%
% Geometry:
%   L1  : tractor wheelbase
%   L1c : hitch offset from tractor rear axle
%   L2  : trailer length from hitch to trailer axle

    % Parameters
    L1  = parameters.L1;
    L1c = parameters.L1c;
    L2  = parameters.L2;
    Ts  = parameters.Ts;

    % Current states
    x2    = X(1);
    y2    = X(2);
    psi2  = wrapToPi(X(3));
    gamma = wrapToPi(X(4));

    % Reconstruct tractor heading
    psi1 = wrapToPi(psi2 + gamma);

    % Truck-trailer kinematics
    x2_dot    = ux * cos(psi1) * cos(gamma) + (L1c / L2) * ux * sin(psi1) * sin(gamma);
    y2_dot    = ux * sin(psi1) * cos(gamma) - (L1c / L2) * ux * cos(psi1) * sin(gamma);
    psi2_dot  = (ux / L2) * sin(gamma) - (L1c / L2) * (ux / L1) * tan(delta) * cos(gamma);
    gamma_dot = (ux / L1) * tan(delta) - psi2_dot;

    % Forward Euler integration
    x2_next    = x2    + Ts * x2_dot;
    y2_next    = y2    + Ts * y2_dot;
    psi2_next  = wrapToPi(psi2  + Ts * psi2_dot);
    gamma_next = wrapToPi(gamma + Ts * gamma_dot);

    % Return next state
    X_next = [x2_next; y2_next; psi2_next; gamma_next];

end