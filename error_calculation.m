function [errors, X_ref0, Y_ref0, theta_ref0, i0] = error_calculation(X0, label, x_r, y_r, theta_r, label_r)

% Error calculation block
%
% Inputs:
%   X0      : current state [X2; Y2; psi2; gamma]
%   label   : current path segment label
%   x_r     : path x coordinates
%   y_r     : path y coordinates
%   theta_r : path heading
%   label_r : path labels
%
% Outputs:
%   errors     : current tracking error magnitude
%   X_ref0     : current closest-point x coordinate
%   Y_ref0     : current closest-point y coordinate
%   theta_ref0 : current closest-point heading
%   i0         : closest-point global index

% Current state components
X2 = X0(1);
Y2 = X0(2);

% Closest point on active segment
[theta_ref0, errors, S_ref, i0] = distance_calc(X2, Y2, label, x_r, y_r, theta_r, label_r);

% Unpack closest-point reference
X_ref0 = S_ref(1);
Y_ref0 = S_ref(2);

end