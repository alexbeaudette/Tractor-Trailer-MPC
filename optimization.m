function U = optimization(Ad, Bd, Wd, Q, R, P, N, U0, X0, X_ref, umin, umax, delta_umin, delta_umax)

% MPC optimization solver
%
% Solves the finite-horizon QP for the affine discrete-time model:
%
%   x(k+1) = Ad*x(k) + Bd*u(k) + Wd
%
% Inputs:
%   Ad, Bd, Wd   : discrete-time affine model
%   Q            : state tracking weight
%   R            : control effort weight
%   P            : control increment weight
%   N            : prediction horizon
%   U0           : previous steering input
%   X0           : current state
%   X_ref        : stacked reference trajectory over horizon
%   umin, umax   : steering bounds
%   delta_umin   : lower bound on steering increment
%   delta_umax   : upper bound on steering increment
%
% Output:
%   U            : stacked optimal control sequence [u(1); u(2); ...; u(N)]

nu = size(Bd,2);

% Build cost matrices (Hessian and gradient vector)
[H, f] = system_metrics(Ad, Bd, Wd, Q, R, P, N, U0, X0, X_ref);

% Build inequality constraints and input bounds
[Aineq, lbA, ubA, lb_u, ub_u] = system_constraints_qpOASES(Ad, Bd, N, U0, umin, umax, delta_umin, delta_umax);

% Solve QP
U = qpOASES(H, f, Aineq, lb_u, ub_u, lbA, ubA);

% Fallback in case solver fails
if isempty(U)
    U = zeros(N*nu,1);
end

end

function [H, f] = system_metrics(Ad, Bd, Wd, Q, R, P, N, U0, X0, X_ref)
% Build the quadratic cost matrices for the MPC problem.

nx = size(Ad,1);
nu = size(Bd,2);

Ax = zeros(N*nx, nx);
for i = 1:N
    Ax((i-1)*nx+1:i*nx, :) = Ad^i;
end

Bu = zeros(N*nx, N*nu);
for i = 1:N
    for j = 1:i
        Bu((i-1)*nx+1:i*nx, (j-1)*nu+1:j*nu) = Ad^(i-j) * Bd;
    end
end

D = zeros(N*nx, 1);
current_sum = zeros(nx, 1);
for i = 1:N
    current_sum = current_sum + Ad^(i-1) * Wd;
    D((i-1)*nx+1:i*nx) = current_sum;
end

T = eye(N*nu) - diag(ones((N-1)*nu, 1), -nu);
t0 = zeros(N*nu, 1);
t0(1:nu) = U0;

Qbar = kron(eye(N), Q);
Rbar = kron(eye(N), R);
Pbar = kron(eye(N), P);

H = 2 * (Bu' * Qbar * Bu + Rbar + T' * Pbar * T);
H = 0.5 * (H + H.');
f = 2 * (Bu' * Qbar * (Ax*X0 + D - X_ref) - T' * Pbar * t0);
end

function [Aineq, lbA, ubA, lb_u, ub_u] = system_constraints_qpOASES(~, Bd, N, U0, umin, umax, delta_umin, delta_umax)
% Build steering magnitude and rate constraints for qpOASES.

nu = size(Bd,2);

T = eye(N*nu) - diag(ones((N-1)*nu,1), -nu);
t0 = zeros(N*nu, 1);
t0(1:nu) = U0;

lb_u = repmat(umin, N, 1);
ub_u = repmat(umax, N, 1);

lb_delta_u = repmat(delta_umin, N, 1);
ub_delta_u = repmat(delta_umax, N, 1);

Aineq = T;
lbA = lb_delta_u + t0;
ubA = ub_delta_u + t0;
end
