function [Ad, Bd, Wd] = truck_trailer_discretization(A, B, W, nu, nx, Ts)
% Discretizes the affine continuous-time model:
%   xdot = A x + B u + W
%
% into:
%   x(k+1) = Ad x(k) + Bd u(k) + Wd

sysc = ss(A, [B W], eye(nx), zeros(nx, nu + 1));
sysd = c2d(sysc, Ts);

Ad = zeros(nx,nx);
BBd = zeros(nx,nu + 1);

[Ad, BBd, ~, ~] = ssdata(sysd);

Bd = BBd(:,1:nu);
Wd = BBd(:,nu + 1:end);

end