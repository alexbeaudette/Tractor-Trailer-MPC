function ref = reference_generator(X0, Ad, Bd, Wd, U0, ux, label, x_r, y_r, theta_r, label_r, dir_r, L1, Llook, N)
%REFERENCE_GENERATOR Build current and horizon references for all states.

[errors, X_ref0, Y_ref0, theta_ref0, i0] = error_calculation(X0, label, x_r, y_r, theta_r, label_r);
[X_des, Y_des, theta_des] = predictive_horizon(X0, Ad, Bd, Wd, U0, label, x_r, y_r, theta_r, label_r, N);
[gamma_ref, theta_h, s_rem, sT, iT, theta_goal, theta_pathT, ...
    heading_err_ref, ref_mode, X_refT, Y_refT] = solve_gamma_des( ...
    X0(1), X0(2), X0(3), ux, i0, label, x_r, y_r, theta_r, label_r, dir_r, L1, Llook);

ref = struct( ...
    'errors', errors, ...
    'X_ref0', X_ref0, ...
    'Y_ref0', Y_ref0, ...
    'theta_ref0', theta_ref0, ...
    'i0', i0, ...
    'X_des', X_des, ...
    'Y_des', Y_des, ...
    'theta_des', theta_des, ...
    'gamma_ref', gamma_ref, ...
    'theta_h', theta_h, ...
    's_rem', s_rem, ...
    'sT', sT, ...
    'iT', iT, ...
    'theta_goal', theta_goal, ...
    'theta_pathT', theta_pathT, ...
    'heading_err_ref', heading_err_ref, ...
    'ref_mode', ref_mode, ...
    'X_refT', X_refT, ...
    'Y_refT', Y_refT);
end

function [errors, X_ref0, Y_ref0, theta_ref0, i0] = error_calculation(X0, label, x_r, y_r, theta_r, label_r)
X2 = X0(1);
Y2 = X0(2);

[theta_ref0, errors, S_ref, i0] = distance_calc(X2, Y2, label, x_r, y_r, theta_r, label_r);
X_ref0 = S_ref(1);
Y_ref0 = S_ref(2);
end

function [theta_ref, distance, S_ref, i0] = distance_calc(x_point, y_point, label, x_r, y_r, theta_r, labels_r)
idx = find(labels_r == label);

if isempty(idx)
    theta_ref = 0;
    distance = inf;
    S_ref = [x_point; y_point];
    i0 = 1;
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

function [X_des, Y_des, theta_des] = predictive_horizon(X0, Ad, Bd, Wd, U0, label, x_r, y_r, theta_r, label_r, N)
Xk = X0;

X_des = zeros(N,1);
Y_des = zeros(N,1);
theta_des = zeros(N,1);

if ~any(label_r == label)
    return
end

for k = 1:N
    Xk = Ad * Xk + Bd * U0 + Wd;
    [theta_ref, ~, S_ref, ~] = distance_calc(Xk(1), Xk(2), label, x_r, y_r, theta_r, label_r);
    X_des(k) = S_ref(1);
    Y_des(k) = S_ref(2);
    theta_des(k) = theta_ref;
end
end

function [gamma_ref, theta_h, s_rem, sT, iT, theta_goal, theta_pathT, ...
    heading_err_ref, ref_mode, X_refT, Y_refT] = solve_gamma_des( ...
    x2, y2, psi2, ux, i0, label, x_r, y_r, theta_r, label_r, dir_r, L1, Llook)

psi2 = wrapToPi(psi2);

gamma_ref = 0;
theta_h = 0;
s_rem = 0;
sT = 0;
iT = i0;
theta_goal = 0;
theta_pathT = 0;
heading_err_ref = 0;
ref_mode = 0;
X_refT = x2;
Y_refT = y2;

idx = find(label_r == label);
if isempty(idx)
    return
end

i0_local = find(idx == i0, 1);
if isempty(i0_local)
    return
end

x_seg = x_r(idx);
y_seg = y_r(idx);
theta_seg = theta_r(idx);

if numel(x_seg) < 2
    return
end

ds = sqrt(diff(x_seg).^2 + diff(y_seg).^2);
s_seg = [0; cumsum(ds)];
s0 = s_seg(i0_local);

dir_cmd = dir_r(i0);
if dir_cmd == 0
    dir_cmd = sign(ux);
end
if dir_cmd == 0
    dir_cmd = 1;
end

theta_idx = segment_index_heading(x_seg, y_seg, i0_local);
heading_alignment = sign(cos(wrapToPi(theta_seg(i0_local) - theta_idx)));
if heading_alignment == 0
    heading_alignment = 1;
end

progress_sign = dir_cmd * heading_alignment;
if progress_sign == 0
    progress_sign = 1;
end

sT = s0 + progress_sign * Llook;
sT = max(min(sT, s_seg(end)), s_seg(1));

[~, iT_local] = min(abs(s_seg - sT));
iT = idx(iT_local);

x_ref = x_r(iT);
y_ref = y_r(iT);
theta_pathT = wrapToPi(theta_r(iT));
X_refT = x_ref;
Y_refT = y_ref;

theta_goal = atan2(y_ref - y2, x_ref - x2);
theta_h = wrapToPi(theta_goal - psi2);
heading_err_ref = theta_h;
ref_mode = progress_sign;

if progress_sign >= 0
    s_rem = s_seg(end) - s0;
else
    s_rem = s0 - s_seg(1);
end
s_rem = max(0, s_rem);

if s_rem < Llook
    gamma_ref = 0;
else
    gamma_ref = atan2(2 * L1 * sin(theta_h), Llook);
end

gamma_ref_max = deg2rad(45);
gamma_ref = max(min(gamma_ref, gamma_ref_max), -gamma_ref_max);
end

function theta_idx = segment_index_heading(x_seg, y_seg, i_local)
if i_local < numel(x_seg)
    dx = x_seg(i_local + 1) - x_seg(i_local);
    dy = y_seg(i_local + 1) - y_seg(i_local);
else
    dx = x_seg(i_local) - x_seg(i_local - 1);
    dy = y_seg(i_local) - y_seg(i_local - 1);
end

theta_idx = atan2(dy, dx);
end
