function [A, B, W, Ad, Bd, Wd] = truck_trailer_model(x0, delta0, u1, L1, L1c, L2, nu, nx, Ts)
%TRUCK_TRAILER_MODEL Build continuous and discrete affine model matrices.
%   Returns:
%     xdot   = A*x + B*u + W
%     x(k+1) = Ad*x(k) + Bd*u(k) + Wd

psi2  = wrapToPi(x0(3));
gamma = wrapToPi(x0(4));

A = A_Matrix(L1, L1c, L2, delta0, psi2, gamma, u1);
B = B_Matrix(L1, L1c, L2, delta0, psi2, gamma, u1);
W = W_Matrix(L1, L1c, L2, delta0, psi2, gamma, u1);
[Ad, Bd, Wd] = discretize_model(A, B, W, nu, nx, Ts);
end

function [Ad, Bd, Wd] = discretize_model(A, B, W, nu, nx, Ts)
% Discretize the affine continuous-time model xdot = A*x + B*u + W.

sysc = ss(A, [B W], eye(nx), zeros(nx, nu + 1));
sysd = c2d(sysc, Ts);

[Ad, BBd, ~, ~] = ssdata(sysd);
Bd = BBd(:,1:nu);
Wd = BBd(:,nu + 1:end);
end

function A = A_Matrix(L1,L1c,L2,delta0,psi20,gamma0,u1)
% Generated symbolic Jacobian d(f)/d(x) at the operating point.

t2 = cos(gamma0);
t3 = cos(psi20);
t4 = tan(delta0);
t5 = sin(gamma0);
t6 = sin(psi20);
t9 = 1.0./L1;
t10 = 1.0./L2;
t7 = L1.*t2;
t8 = L1.*t5;
t11 = L1c.*t2.*t4;
t12 = L1c.*t4.*t5;
t13 = -t12;
t14 = t8+t11;
t15 = t7+t13;
t16 = t9.*t10.*t15.*u1;
A = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-t6.*t9.*t15.*u1,t3.*t9.*t15.*u1,0.0,0.0,-t3.*t9.*t14.*u1,-t6.*t9.*t14.*u1,t16,-t16],[4,4]);
end

function B = B_Matrix(L1,L1c,L2,delta0,psi20,gamma0,u1)
% Generated symbolic Jacobian d(f)/d(u) at the operating point.

t2 = cos(delta0);
t3 = cos(gamma0);
t4 = sin(gamma0);
t5 = 1.0./L1;
t6 = 1.0./L2;
t7 = 1.0./t2.^2;
B = [-L1c.*t4.*t5.*t7.*u1.*cos(psi20);(L1c.*t4.*t5.*u1.*sin(psi20))./(sin(delta0).^2-1.0);L1c.*t3.*t5.*t6.*t7.*u1;t5.*t6.*t7.*u1.*(L2-L1c.*t3)];
end

function W = W_Matrix(L1,L1c,L2,delta0,psi20,gamma0,u1)
% Generated symbolic affine offset term at the operating point.

t2 = cos(delta0);
t3 = cos(gamma0);
t4 = cos(psi20);
t5 = sin(delta0);
t6 = tan(delta0);
t7 = sin(gamma0);
t8 = sin(psi20);
t12 = 1.0./L1;
t13 = 1.0./L2;
t9 = t2.^2;
t10 = L1.*t3;
t11 = L1.*t7;
t15 = L1c.*delta0.*t3;
t16 = L1c.*t3.*t6;
t17 = L1c.*t6.*t7;
t19 = L1c.*t2.*t3.*t5;
t23 = L1c.*gamma0.*t2.*t5.*t7;
t14 = 1.0./t9;
t18 = -t15;
t20 = -t17;
t21 = t9.*t11;
t22 = gamma0.*t9.*t10;
t25 = t11+t16;
t24 = -t22;
t26 = t10+t20;
W = [t4.*t12.*t26.*u1+gamma0.*t4.*t12.*t25.*u1+psi20.*t8.*t12.*t26.*u1+L1c.*delta0.*t4.*t7.*t12.*t14.*u1;t8.*t12.*t26.*u1+gamma0.*t8.*t12.*t25.*u1-psi20.*t4.*t12.*t26.*u1-(L1c.*delta0.*t7.*t8.*t12.*u1)./(t5.^2-1.0);t12.*t13.*t14.*u1.*(t18+t19+t21+t23+t24);-t12.*t13.*t14.*u1.*(t18+t19+t21+t23+t24+L2.*delta0-L2.*t2.*t5)];
end

function f_trailer = f_tractor_trailer(~,~,psi2,gamma,delta,u1,L1,L1c,L2) %#ok<DEFNU>
% Generated symbolic nonlinear trailer-state dynamics.

t2 = cos(gamma);
t3 = tan(delta);
t4 = sin(gamma);
t6 = 1.0./L1;
t7 = 1.0./L2;
t5 = L1.*t2;
t8 = L1c.*t3.*t4;
t9 = t4.*t7.*u1;
t12 = L1c.*t2.*t3.*t6.*t7.*u1;
t10 = -t8;
t11 = t5+t10;
f_trailer = [t6.*t11.*u1.*cos(psi2);t6.*t11.*u1.*sin(psi2);t9+t12;-t9-t12+t3.*t6.*u1];
end
