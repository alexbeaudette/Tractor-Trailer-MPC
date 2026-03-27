function [theta_ref, distance, S_ref, i0] = distance_calc(x_point, y_point, label, x_r, y_r, theta_r, labels_r)

% Distance calculation / path association
%
% Inputs:
%   x_point  : queried x position
%   y_point  : queried y position
%   label    : active path segment label
%   x_r      : reference path x coordinates
%   y_r      : reference path y coordinates
%   theta_r  : reference path heading
%   labels_r : reference path labels
%   
% Outputs:
%   theta_ref : heading at closest reference point
%   distance  : distance to closest reference point
%   S_ref     : closest reference point on the path [x; y]

idx = find(labels_r == label);

if isempty(idx)
    theta_ref = 0;
    distance  = inf;
    S_ref     = [x_point; y_point];
    i0  = 1;
    return
end

x_seg = x_r(idx);
y_seg = y_r(idx);
th_seg = theta_r(idx);

distances = sqrt((x_seg - x_point).^2 + (y_seg - y_point).^2);

[distance, k] = min(distances);

theta_ref = th_seg(k);
S_ref = [x_seg(k); y_seg(k)];
i0 = idx(k);

end