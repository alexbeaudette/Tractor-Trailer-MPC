%  SYMBOLIC DERIVATION FOR TRUCK-TRAILER KINEMATIC MODEL
%
%  State definition:
%    x = [X2; Y2; psi2; gamma]
%
%  where:
%    X2, Y2 : trailer rear axle position in global frame
%    psi2   : trailer yaw
%    gamma  : hitch angle = psi1 - psi2
%
%  Input:
%    delta  : truck steering angle
%
%  Known input / scheduling variable:
%    u1     : truck longitudinal speed

%% ---------------- Symbolic variables ----------------
syms X1 Y1 X2 Y2 psi1 psi2 gamma real
syms delta u1 real
syms L1 L1c L2 real

% Operating point variables for linearization
syms X20 Y20 psi20 gamma0 delta0 real

%% ---------------- Trailer-based state ----------------
x_trailer = [X2; Y2; psi2; gamma];
u = delta;

% Reconstruct truck yaw
psi1 = psi2 + gamma;

%% ---------------- Truck nonlinear model ----------------
X1_dot   = u1*cos(psi1);
Y1_dot   = u1*sin(psi1);
psi1_dot = (u1/L1)*tan(delta);

%% ---------------- Trailer yaw / articulation dynamics ----------------
psi2_dot  = (u1/L2)*sin(gamma) + (L1c/L2)*(u1/L1)*tan(delta)*cos(gamma);
gamma_dot = psi1_dot - psi2_dot;

%% ---------------- Geometry in trailer frame (2) ----------------
% Hitch point
Xh_2 = X2 + L2*cos(psi2);
Yh_2 = Y2 + L2*sin(psi2);

% Truck rear axle
X1_2 = Xh_2 - L1c*cos(psi1);
Y1_2 = Yh_2 - L1c*sin(psi1);

%% ---------------- Recover trailer position rates from geometry ----------------
% Since X1_2 and Y1_2 represent the truck rear axle in terms of trailer states,
% enforce consistency with the truck kinematics to solve for X2_dot and Y2_dot.

syms X2_dot Y2_dot real

X1_dot_geom = diff(X1_2, X2)*X2_dot + diff(X1_2, Y2)*Y2_dot + ...
              diff(X1_2, psi2)*psi2_dot + diff(X1_2, gamma)*gamma_dot;

Y1_dot_geom = diff(Y1_2, X2)*X2_dot + diff(Y1_2, Y2)*Y2_dot + ...
              diff(Y1_2, psi2)*psi2_dot + diff(Y1_2, gamma)*gamma_dot;

sol = solve([X1_dot_geom == X1_dot, Y1_dot_geom == Y1_dot], [X2_dot, Y2_dot]);

X2_dot = simplify(sol.X2_dot);
Y2_dot = simplify(sol.Y2_dot);

%% ---------------- Trailer nonlinear model ----------------
f_trailer = [X2_dot;
             Y2_dot;
             psi2_dot;
             gamma_dot];

%% ---------------- Linearization in trailer frame (2) ----------------
x0 = [X20; Y20; psi20; gamma0];
u0 = delta0;

A_sym = jacobian(f_trailer, x_trailer);
B_sym = jacobian(f_trailer, u);

A = simplify(subs(A_sym, [x_trailer; u], [x0; u0]));
B = simplify(subs(B_sym, [x_trailer; u], [x0; u0]));
W = simplify(subs(f_trailer, [x_trailer; u], [x0; u0]) - A*x0 - B*u0);

%% ---------------- Generate MATLAB functions ----------------

matlabFunction(A, 'File', 'A_Matrix', ...
    'Vars', {L1, L1c, L2, delta0, psi20, gamma0, u1});

matlabFunction(B, 'File', 'B_Matrix', ...
    'Vars', {L1, L1c, L2, delta0, psi20, gamma0, u1});

matlabFunction(W, 'File', 'W_Matrix', ...
    'Vars', {L1, L1c, L2, delta0, psi20, gamma0, u1});

matlabFunction(f_trailer, 'File', 'f_tractor_trailer', ...
    'Vars', {X2, Y2, psi2, gamma, delta, u1, L1, L1c, L2});

disp('Symbolic derivation complete.');