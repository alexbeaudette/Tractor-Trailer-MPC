function [Aineq, lbA, ubA, lb_u, ub_u] = system_constraints_qpOASES(Ad, Bd, N, U0, umin, umax, delta_umin, delta_umax)

% Builds MPC input and steering-rate constraints
%
% Inputs:
%   Ad, Bd        : discrete-time model matrices
%   N             : prediction horizon
%   U0            : previous steering input
%   umin, umax    : steering bounds
%   delta_umin    : minimum steering increment
%   delta_umax    : maximum steering increment
%
% Outputs:
%   Aineq         : matrix used for delta-u constraints
%   lbA, ubA      : lower/upper bounds on Aineq*U
%   lb_u, ub_u    : lower/upper bounds on U directly

nu = size(Bd,2);

% Construct T (converts stacked steering inputs into steering increments)
T = eye(N*nu) - diag(ones((N-1)*nu,1), -nu);

% Construct t0 (accounts for previous applied steering input U0)
t0 = zeros(N*nu, 1);
t0(1:nu) = U0;

% Absolute steering bounds
lb_u = repmat(umin, N, 1);
ub_u = repmat(umax, N, 1);

% Steering increment bounds
lb_delta_u = repmat(delta_umin, N, 1);
ub_delta_u = repmat(delta_umax, N, 1);

% Constraint form for qpOASES
% lbA <= Aineq*U <= ubA
Aineq = T;
lbA = lb_delta_u + t0;
ubA = ub_delta_u + t0;

end