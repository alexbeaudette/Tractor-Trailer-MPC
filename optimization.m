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

nx = size(Ad,1);
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