function meas = get_measurements(X, ux, L1c, L2)
%
% Inputs:
%   X   : current state vector
%         X = [x2; y2; psi2; gamma]
%   ux  : longitudinal speed [m/s]
%   L1c : hitch offset relative to tractor rear axle [m]
%   L2  : trailer length from hitch to trailer axle [m]
%
% Outputs:
%   meas : struct containing the reconstructed measurement signals
%          meas.x2
%          meas.y2
%          meas.psi2
%          meas.gamma
%          meas.psi1
%          meas.x1
%          meas.y1
%          meas.ux
%          meas.X
%
% Notes:
%   - x2, y2 are the trailer rear axle position
%   - psi2 is the trailer yaw
%   - gamma = psi1 - psi2
%   - psi1 is reconstructed from psi2 and gamma
%   - x1, y1 are the tractor rear axle position

    % Extract position states
    x2 = X(1);
    y2 = X(2);

    % Enforce angular consistency
    psi2  = wrapToPi(X(3));
    gamma = wrapToPi(X(4));
    psi1  = wrapToPi(psi2 + gamma);

    % Reconstruct hitch position
    xh = x2 + L2*cos(psi2);
    yh = y2 + L2*sin(psi2);

    % Reconstruct tractor rear axle position
    x1 = xh - L1c*cos(psi1);
    y1 = yh - L1c*sin(psi1);

    % Pack outputs
    meas = struct();

    meas.x1    = x1;
    meas.y1    = y1;
    meas.x2    = x2;
    meas.y2    = y2;
    meas.xh    = xh;
    meas.yh    = yh;
    meas.psi1  = psi1;
    meas.psi2  = psi2;
    meas.gamma = gamma;
    meas.ux    = ux;
    meas.X     = [x2; y2; psi2; gamma];

end