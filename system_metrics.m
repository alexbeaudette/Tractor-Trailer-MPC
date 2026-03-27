function [H, f] = system_metrics(Ad, Bd, Wd, Q, R, P, N, U0, X0, X_ref)

% Builds the quadratic cost matrices for the MPC problem
%
% Predicted model over the horizon:
%
%   X = Ax*X0 + Bu*U + D
%
% where:
%   X  = stacked predicted states
%   U  = stacked control inputs
%   D  = stacked affine disturbance/offset contribution from Wd
%
% Cost used:
%
%   J = tracking error cost + control effort cost + control increment cost
%
% Inputs:
%   Ad, Bd, Wd : discrete-time affine model
%   Q          : state tracking weight
%   R          : control effort weight
%   P          : control increment weight
%   N          : prediction horizon
%   U0         : previous control input
%   X0         : current state
%   X_ref      : stacked state reference over horizon
%
% Outputs:
%   H, f       : Hessian and gradient for the QP

nx = size(Ad,1);
nu = size(Bd,2);

% Construct Ax (maps current state X0 to stacked future states)
Ax = zeros(N*nx, nx);
for i = 1:N
    Ax((i-1)*nx+1:i*nx, :) = Ad^i;
end

% Construct Bu (maps stacked control sequence U to stacked future states)
Bu = zeros(N*nx, N*nu);
for i = 1:N
    for j = 1:i
        Bu((i-1)*nx+1:i*nx, (j-1)*nu+1:j*nu) = Ad^(i-j) * Bd;
    end
end

% Construct D (maps the affine term Wd over the horizon)
D = zeros(N*nx, 1);
current_sum = zeros(nx, 1);

for i = 1:N
    current_sum = current_sum + Ad^(i-1) * Wd;
    D((i-1)*nx+1:i*nx) = current_sum;
end

% Construct T (maps stacked inputs U into stacked input increments DeltaU)
%
% DeltaU = T*U - t0
%
% where t0 contains the previous input U0 in the first block
T = eye(N*nu) - diag(ones((N-1)*nu, 1), -nu);

% Construct t0
t0 = zeros(N*nu, 1);
t0(1:nu) = U0;

% Build Hessian and gradient
% Cost:
%   1) state tracking with Q
%   2) input effort with R
%   3) input increment penalty with P

Qbar = kron(eye(N), Q);
Rbar = kron(eye(N), R);
Pbar = kron(eye(N), P);

H = 2 * (Bu' * Qbar * Bu + Rbar + T' * Pbar * T);
H = 0.5 * (H + H.');   % enforce symmetry

f = 2 * (Bu' * Qbar * (Ax*X0 + D - X_ref) - T' * Pbar * t0);

end