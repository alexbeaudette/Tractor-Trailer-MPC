function [gamma_ref, theta_h, s_rem, sT, iT, theta_goal] = solve_gamma_des( ...
    x2, y2, psi2, ux, i0, label, x_r, y_r, theta_r, label_r, dir_r, L1, Llook)

% Compute desired hitch angle
%
% Inputs:
%   x2, y2   : current trailer position
%   psi2     : current trailer heading
%   ux       : current longitudinal velocity
%   i0       : closest-point global index on full path
%   label    : active path segment label
%   x_r      : reference path x coordinates
%   y_r      : reference path y coordinates
%   theta_r  : reference path heading
%   label_r  : reference path labels
%   dir_r    : reference path direction field
%   L1       : truck wheelbase
%   Llook    : lookahead distance
%
% Outputs:
%   gamma_ref  : desired hitch angle
%   theta_h    : heading from trailer to target, relative to trailer heading
%   s_rem      : remaining distance in commanded direction
%   sT         : lookahead station
%   iT         : lookahead target global index
%   theta_goal : world-frame angle from trailer to lookahead target

    psi2 = wrapToPi(psi2);
    
    % Defaults
    gamma_ref  = 0;
    theta_h    = 0;
    s_rem      = 0;
    sT         = 0;
    iT         = i0;
    theta_goal = 0;

    % Active segment indices
    idx = find(label_r == label);
    if isempty(idx)
        return
    end

    % Convert global closest-point index -> local index on active segment
    i0_local = find(idx == i0, 1);
    if isempty(i0_local)
        return
    end

    % Active segment data
    x_seg     = x_r(idx);
    y_seg     = y_r(idx);
    theta_seg = theta_r(idx);

    if numel(x_seg) < 2
        return
    end

    % Build cumulative station for active segment
    ds    = sqrt(diff(x_seg).^2 + diff(y_seg).^2);
    s_seg = [0; cumsum(ds)];
    s0    = s_seg(i0_local);

    % Local path tangent at closest point
    theta0 = theta_seg(i0_local);
    t_hat  = [cos(theta0); sin(theta0)];
    v_hat  = [cos(psi2);   sin(psi2)];

    % Same logic as LQR
    dirSign = sign(t_hat.' * v_hat);
    if dirSign == 0
        dirSign = 1;
    end

    motionSign = sign(ux);
    if motionSign == 0
        motionSign = 1;
    end

    % Lookahead station
    sT = s0 + (motionSign * dirSign) * Llook;
    sT = max(min(sT, s_seg(end)), s_seg(1));

    % Target index on active segment, then convert back to global
    [~, iT_local] = min(abs(s_seg - sT));
    iT = idx(iT_local);

    % Lookahead target point
    x_ref = x_r(iT);
    y_ref = y_r(iT);

    % Angle from trailer to lookahead target
    dxT = x_ref - x2;
    dyT = y_ref - y2;
    theta_goal = atan2(dyT, dxT);

    % Same relative-angle logic as LQR
    theta_h = wrapToPi(theta_goal - psi2);

    % Remaining distance logic from LQR
    dir_cmd = dir_r(i0);
    sigma   = dir_cmd * dirSign;

    if sigma >= 0
        s_rem = s_seg(end) - s0;
    else
        s_rem = s0 - s_seg(1);
    end

    s_rem = max(0, s_rem);

    % Desired hitch angle
    if s_rem < Llook
        gamma_ref = 0;
    else
        gamma_ref = atan2(2 * L1 * sin(theta_h), Llook);
    end

    % Saturation
    gamma_ref_max = deg2rad(45);
    gamma_ref = max(min(gamma_ref, gamma_ref_max), -gamma_ref_max);

end