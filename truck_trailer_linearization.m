function [A, B, W] = truck_trailer_linearization(x0, delta0, u1, L1, L1c, L2)
% Returns continuous-time affine linearization matrices
% State: x = [X2; Y2; psi2; gamma]

X2    = x0(1); %#ok<NASGU>
Y2    = x0(2); %#ok<NASGU>
psi2  = wrapToPi(x0(3));
gamma = wrapToPi(x0(4));

A = A_Matrix(L1, L1c, L2, delta0, psi2, gamma, u1);
B = B_Matrix(L1, L1c, L2, delta0, psi2, gamma, u1);
W = W_Matrix(L1, L1c, L2, delta0, psi2, gamma, u1);

end