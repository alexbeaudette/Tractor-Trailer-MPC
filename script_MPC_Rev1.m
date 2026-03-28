%% MPC Script version - Rev1
% hit 'Run Section' for top section of this script to initialize parameters,
% path generation, and add paths to necessary functions
% clear; clc; close all;

base_path = 'C:\Electrans\Research\MPC\MPC_script_monkey';

addpath(fullfile(base_path, 'qpOASES-slim'))

%% Vehicle / simulation settings
ux = -1.0;                     % constant longitudinal speed [m/s]
Nsim = 1000;                   % number of simulation steps

%% Path definition
path_type = "line";           % "merge", "line", "circle", "figure8", "parkingfr"

% Initial path anchoring
x_start = -13.006;
y_start = -2;
theta_path = -15;

% Generate path
path = path_generation(path_type, ux, x_start, y_start, theta_path);

x_r     = path(:,1);
y_r     = path(:,2);
theta_r = path(:,3);
s_r     = path(:,4);
kappa_r = path(:,5);
dir_r   = path(:,6);
label_r = path(:,7);

%% Initial state
% X = [X2; Y2; psi2; gamma]
X = zeros(4,1);
X(1) = -13.261;
X(2) = 0;
X(3) = deg2rad(0);
X(4) = deg2rad(0);

delta_prev = 0;
prev_label = 1;
switch_flag = 0;

%% Parameter Initialization from class
L1         = parameters.L1;
L1c        = parameters.L1c;
L2         = parameters.L2;
Llook      = parameters.Llook;
Ts         = parameters.Ts;
nx         = parameters.nx;
nu         = parameters.nu;
N          = parameters.N;
umin       = parameters.umin;
umax       = parameters.umax;
delta_umin = parameters.delta_umin;
delta_umax = parameters.delta_umax;
Q_fwd      = parameters.Q_fwd;
Q_rev      = parameters.Q_rev;
R          = parameters.R;
P          = parameters.P;

%% Initial debug plot
[~, label0, ~] = segment_manager( ...
    X, x_r, y_r, theta_r, label_r, dir_r, prev_label, switch_flag);
[~, ~, ~, Ad0, Bd0, Wd0] = truck_trailer_model( ...
    X, delta_prev, ux, L1, L1c, L2, nu, nx, Ts);
ref0 = reference_generator( ...
    X, Ad0, Bd0, Wd0, delta_prev, ux, label0, ...
    x_r, y_r, theta_r, label_r, dir_r, L1, Llook, N);
meas0 = get_measurements(X, ux, L1c, L2);

x2_0 = meas0.x2;
y2_0 = meas0.y2;
psi2_0 = meas0.psi2;
x1_0 = meas0.x1;
y1_0 = meas0.y1;
psi1_0 = meas0.psi1;
X_ref0_init = ref0.X_ref0;
Y_ref0_init = ref0.Y_ref0;
theta_ref0_init = ref0.theta_ref0;
psi_err0 = wrapToPi(psi2_0 - theta_ref0_init);

trailer_len = L2;
tractor_len = L1;
trailer_w = 2.5;
tractor_w = 2.5;
arrow_len = 2.2;

c2x_0 = x2_0 + (trailer_len/2) * cos(psi2_0);
c2y_0 = y2_0 + (trailer_len/2) * sin(psi2_0);
[Xt0, Yt0] = get_box_coords(c2x_0, c2y_0, psi2_0, trailer_len, trailer_w);

c1x_0 = x1_0 + (tractor_len/2) * cos(psi1_0);
c1y_0 = y1_0 + (tractor_len/2) * sin(psi1_0);
[Xc0, Yc0] = get_box_coords(c1x_0, c1y_0, psi1_0, tractor_len, tractor_w);

figure('Color','k', 'Name', 'Initial Debug');
ax0 = gca;
set(ax0, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
hold on; grid on; axis equal;

h_refpath0 = plot(x_r, y_r, '--', 'Color', [0.75 0.75 0.75], 'LineWidth', 1.4);
h_trailer0 = patch('XData', Xt0, 'YData', Yt0, ...
    'FaceColor', 'none', 'EdgeColor', [0.92 0.92 0.92], 'LineWidth', 1.8);
h_tractor0 = patch('XData', Xc0, 'YData', Yc0, ...
    'FaceColor', [0.20 0.80 1.00], 'FaceAlpha', 0.22, ...
    'EdgeColor', [0.70 0.95 1.00], 'LineWidth', 1.6);
h_start0 = plot(x2_0, y2_0, 'o', ...
    'MarkerSize', 6, 'MarkerFaceColor', [1 1 1], 'MarkerEdgeColor', [1 1 1]);
h_closest0 = plot(X_ref0_init, Y_ref0_init, 'o', ...
    'MarkerSize', 6, 'MarkerFaceColor', [1 0.15 0.15], 'MarkerEdgeColor', [1 0.15 0.15]);
h_line0 = plot([x2_0, X_ref0_init], [y2_0, Y_ref0_init], '-', ...
    'Color', [1.00 0.90 0.20], 'LineWidth', 1.8);
h_heading0 = quiver(x2_0, y2_0, arrow_len*cos(psi2_0), arrow_len*sin(psi2_0), 0, ...
    'Color', [0.10 0.95 1.00], 'LineWidth', 2.0, 'MaxHeadSize', 0.8);
h_tangent0 = quiver(X_ref0_init, Y_ref0_init, ...
    arrow_len*cos(theta_ref0_init), arrow_len*sin(theta_ref0_init), 0, ...
    'Color', [1.00 0.10 1.00], 'LineWidth', 2.0, 'MaxHeadSize', 0.8);

xlabel('X (m)', 'Color', 'w');
ylabel('Y (m)', 'Color', 'w');
title(sprintf('Initial Debug | psi error = %.2f deg', rad2deg(psi_err0)), 'Color', 'w');
legend([h_refpath0, h_trailer0, h_tractor0, h_start0, h_closest0, h_line0, h_heading0, h_tangent0], ...
    {'Reference Path', 'Trailer', 'Tractor', 'Start Trailer Axle', ...
     'Closest Pt', 'Closest-Point Line', 'Trailer Heading', 'Path Tangent'}, ...
    'TextColor', 'w', 'Location', 'northeast');

xmin0 = min([x2_0, x1_0, X_ref0_init, Xt0, Xc0]) - 6;
xmax0 = max([x2_0, x1_0, X_ref0_init, Xt0, Xc0]) + 6;
ymin0 = min([y2_0, y1_0, Y_ref0_init, Yt0, Yc0]) - 4;
ymax0 = max([y2_0, y1_0, Y_ref0_init, Yt0, Yc0]) + 4;
xlim([xmin0 xmax0]);
ylim([ymin0 ymax0]);

%% Logs
logs = struct();

% Measured / reconstructed quantities
logs.x1    = nan(1, Nsim);
logs.y1    = nan(1, Nsim);
logs.x2    = nan(1, Nsim);
logs.y2    = nan(1, Nsim);
logs.xh    = nan(1, Nsim);
logs.yh    = nan(1, Nsim);
logs.psi1  = nan(1, Nsim);
logs.psi2  = nan(1, Nsim);
logs.gamma = nan(1, Nsim);
logs.ux    = nan(1, Nsim);

% State vectors
logs.X      = nan(nx, Nsim);
logs.X_next = nan(nx, Nsim);

% Segment / path association
logs.label = nan(1, Nsim);
logs.mode  = nan(1, Nsim);
logs.i0    = nan(1, Nsim);
logs.iT    = nan(1, Nsim);

% Closest-point reference
logs.X_ref0     = nan(1, Nsim);
logs.Y_ref0     = nan(1, Nsim);
logs.theta_ref0 = nan(1, Nsim);
logs.errors     = nan(1, Nsim);

% Desired/reference quantities
logs.gamma_ref  = nan(1, Nsim);
logs.theta_h    = nan(1, Nsim);
logs.theta_goal = nan(1, Nsim);
logs.s_rem      = nan(1, Nsim);
logs.sT         = nan(1, Nsim);

% Prediction horizon
logs.X_des     = nan(N, Nsim);
logs.Y_des     = nan(N, Nsim);
logs.theta_des = nan(N, Nsim);

% Model matrices
logs.A = nan(nx, nx, Nsim);
logs.B = nan(nx, nu, Nsim);
logs.W = nan(nx, Nsim);
logs.Ad = nan(nx, nx, Nsim);
logs.Bd = nan(nx, nu, Nsim);
logs.Wd = nan(nx, Nsim);

% Optimization outputs
logs.U          = nan(nu*N, Nsim);
logs.u          = nan(nu, N, Nsim);
logs.delta      = nan(1, Nsim);
logs.delta_prev = nan(1, Nsim);

% Debugging outputs
logs.psi_err = nan(1, Nsim);
logs.closest_dist = nan(1, Nsim);

%% Simulation loop
for k = 1:Nsim

    %% Reconstruct measurements from current state
    meas = get_measurements(X, ux, L1c, L2);

    x1    = meas.x1;
    y1    = meas.y1;
    x2    = meas.x2;
    y2    = meas.y2;
    psi1  = meas.psi1;
    psi2  = meas.psi2;
    gamma = meas.gamma;

    %% Segment manager
    [mode, label, switch_flag] = segment_manager( ...
        X, x_r, y_r, theta_r, label_r, dir_r, prev_label, switch_flag);

    %% Linearization and discretization
    [A, B, W, Ad, Bd, Wd] = truck_trailer_model( ...
        X, delta_prev, ux, L1, L1c, L2, nu, nx, Ts);

    %% Reference generation
    ref = reference_generator( ...
        X, Ad, Bd, Wd, delta_prev, ux, label, ...
        x_r, y_r, theta_r, label_r, dir_r, L1, Llook, N);

    errors = ref.errors;
    X_ref0 = ref.X_ref0;
    Y_ref0 = ref.Y_ref0;
    theta_ref0 = ref.theta_ref0;
    i0 = ref.i0;
    X_des = ref.X_des;
    Y_des = ref.Y_des;
    theta_des = ref.theta_des;
    gamma_ref = ref.gamma_ref;
    theta_h = ref.theta_h;
    s_rem = ref.s_rem;
    sT = ref.sT;
    iT = ref.iT;
    theta_goal = ref.theta_goal;

    % Debugging
    psi_err = wrapToPi(psi2 - theta_ref0);
    closest_dist = hypot(x2 - X_ref0, y2 - Y_ref0);

    %% Direction-dependent Q
    if isempty(mode)
        mode = 1;
    end

    if mode < 0
        Q = Q_rev;
    else
        Q = Q_fwd;
    end

    %% Build stacked reference vector
    X_ref = zeros(nx*N,1);
    for i = 1:N
        X_ref(nx*(i-1)+1:nx*i) = [X_des(i); Y_des(i); theta_des(i); gamma_ref];
    end

    %% Solve optimization
    U = optimization(Ad, Bd, Wd, Q, R, P, N, delta_prev, X, X_ref, ...
                     umin, umax, delta_umin, delta_umax);

    u = reshape(U, nu, N);
    delta = u(:,1);

    %% Plant propagation
    X_next = plant_propagation(X, delta, ux);

    %% Logging
    logs.x1(k)    = x1;
    logs.y1(k)    = y1;
    logs.x2(k)    = x2;
    logs.y2(k)    = y2;
    logs.xh(k)    = meas.xh;
    logs.yh(k)    = meas.yh;
    logs.psi1(k)  = psi1;
    logs.psi2(k)  = psi2;
    logs.gamma(k) = gamma;
    logs.ux(k)    = ux;

    logs.X(:,k)      = X;
    logs.X_next(:,k) = X_next;

    logs.label(k) = label;
    logs.mode(k)  = mode;
    logs.i0(k)    = i0;
    logs.iT(k)    = iT;

    logs.X_ref0(k)     = X_ref0;
    logs.Y_ref0(k)     = Y_ref0;
    logs.theta_ref0(k) = theta_ref0;
    logs.errors(k)     = errors;

    logs.X_des(:,k)     = X_des;
    logs.Y_des(:,k)     = Y_des;
    logs.theta_des(:,k) = theta_des;

    logs.gamma_ref(k)  = gamma_ref;
    logs.theta_h(k)    = theta_h;
    logs.theta_goal(k) = theta_goal;
    logs.s_rem(k)      = s_rem;
    logs.sT(k)         = sT;

    logs.A(:,:,k) = A;
    logs.B(:,:,k) = B;
    logs.W(:,k)   = W;
    logs.Ad(:,:,k) = Ad;
    logs.Bd(:,:,k) = Bd;
    logs.Wd(:,k)   = Wd;

    logs.U(:,k)          = U;
    logs.u(:,:,k)        = u;
    logs.delta(k)        = delta;
    logs.delta_prev(k)   = delta_prev;

    % Debugging
    logs.psi_err(k) = psi_err;
    logs.closest_dist(k) = closest_dist;

    %% Update for next iteration
    X = X_next;
    delta_prev = delta;
    prev_label = label;
end

%% Plots

t = (0:Nsim-1) * Ts;
col_ref      = [0.65 0.65 0.65];
col_path     = [0.20 0.80 1.00];
col_heading  = [1.00 0.80 0.20];
col_hitch    = [0.30 1.00 0.50];
col_limit_hi = [1.00 0.45 0.20];
col_limit_lo = [1.00 0.20 0.70];
col_error    = [1.00 0.40 0.40];
col_mode     = [0.70 0.50 1.00];
col_i0       = [0.20 0.90 1.00];
col_iT       = [1.00 0.85 0.25];

% Path tracking
figure('Color','k');
set(gca,'Color','k','XColor','w','YColor','w');
hold on; grid on; axis equal;
h_ref = plot(x_r, y_r, '--', 'Color', col_ref, 'LineWidth', 1.5);
h_path = plot(logs.x2, logs.y2, 'Color', col_path, 'LineWidth', 2);
xlabel('X [m]', 'Color','w');
ylabel('Y [m]', 'Color','w');
title('Path Tracking', 'Color','w');
legend([h_ref, h_path], {'Reference Path','Trailer Path'}, 'TextColor','w','Location','best');

% Trailer heading vs reference
figure('Color','k');
set(gca,'Color','k','XColor','w','YColor','w');
hold on; grid on;
h_psi2 = plot(t, rad2deg(logs.psi2), 'Color', col_path, 'LineWidth', 1.8);
h_theta_ref = plot(t, rad2deg(logs.theta_ref0), '--', 'Color', col_heading, 'LineWidth', 1.5);
xlabel('Time [s]', 'Color','w');
ylabel('Angle [deg]', 'Color','w');
title('Trailer Heading Tracking', 'Color','w');
legend([h_psi2, h_theta_ref], {'\psi_2','\theta_{ref,0}'}, 'TextColor','w','Location','best');
xlim([t(1) t(end)]);

% Hitch angle vs reference
figure('Color','k');
set(gca,'Color','k','XColor','w','YColor','w');
hold on; grid on;
h_gamma = plot(t, rad2deg(logs.gamma), 'Color', col_hitch, 'LineWidth', 1.8);
h_gamma_ref = plot(t, rad2deg(logs.gamma_ref), '--', 'Color', col_heading, 'LineWidth', 1.5);
xlabel('Time [s]', 'Color','w');
ylabel('Angle [deg]', 'Color','w');
title('Hitch Angle Tracking', 'Color','w');
legend([h_gamma, h_gamma_ref], {'\gamma','\gamma_{ref}'}, 'TextColor','w','Location','best');
xlim([t(1) t(end)]);

% Steering input
figure('Color','k');
set(gca,'Color','k','XColor','w','YColor','w');
hold on; grid on;
h_delta = plot(t, rad2deg(logs.delta), 'Color', col_path, 'LineWidth', 1.8);
h_umax = yline(rad2deg(delta_umax), '--', 'Color', col_limit_hi, 'LineWidth', 1.4);
h_umin = yline(rad2deg(delta_umin), ':', 'Color', col_limit_lo, 'LineWidth', 1.8);
xlabel('Time [s]', 'Color','w');
ylabel('Steering [deg]', 'Color','w');
title('Steering Input', 'Color','w');
legend([h_delta, h_umax, h_umin], {'\delta','Upper Limit','Lower Limit'}, 'TextColor','w','Location','best');
xlim([t(1) t(end)]);

% Tracking error
figure('Color','k');
set(gca,'Color','k','XColor','w','YColor','w');
hold on; grid on;
plot(t, logs.errors, 'Color', col_error, 'LineWidth', 1.8);
xlabel('Time [s]', 'Color','w');
ylabel('Error [m]', 'Color','w');
title('Tracking Error', 'Color','w');
xlim([t(1) t(end)]);

% Segment mode
figure('Color','k');
set(gca,'Color','k','XColor','w','YColor','w');
hold on; grid on;
plot(t, logs.mode, 'Color', col_mode, 'LineWidth', 1.8);
xlabel('Time [s]', 'Color','w');
ylabel('Mode', 'Color','w');
title('Segment Direction / Mode', 'Color','w');
xlim([t(1) t(end)]);

% Closest point and lookahead indices
figure('Color','k');
set(gca,'Color','k','XColor','w','YColor','w');
hold on; grid on;
h_i0 = plot(t, logs.i0, 'Color', col_i0, 'LineWidth', 1.8);
h_iT = plot(t, logs.iT, '--', 'Color', col_iT, 'LineWidth', 1.5);
xlabel('Time [s]', 'Color','w');
ylabel('Path Index', 'Color','w');
title('Closest-Point and Lookahead Indices', 'Color','w');
legend([h_i0, h_iT], {'i0','iT'}, 'TextColor','w','Location','best');
xlim([t(1) t(end)]);

%% Animation for Tractor-Trailer Boxes

figure('Name','Tractor-Trailer Animation','Color','k');
grid on; axis equal; hold on;
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');

plot(x_r, y_r, 'w--', 'LineWidth', 1.0);

xlabel('X (m)', 'Color', 'w');
ylabel('Y (m)', 'Color', 'w');
title('Animation: Vehicle Tracking Over Time', 'Color', 'w');

% Initialize graphic objects
h_traj = plot(nan, nan, 'w', 'LineWidth', 1.5);

h_trailer = patch('XData', nan(1,4), 'YData', nan(1,4), ...
                  'FaceColor', [0.7 0.7 0.7], ...
                  'FaceAlpha', 0.35, ...
                  'EdgeColor', 'w', ...
                  'LineWidth', 1.2);

h_tractor = patch('XData', nan(1,4), 'YData', nan(1,4), ...
                  'FaceColor', [0.3 0.8 1.0], ...
                  'FaceAlpha', 0.35, ...
                  'EdgeColor', 'w', ...
                  'LineWidth', 1.2);

% Vehicle dimensions
tractor_len = parameters.L1;
trailer_len = parameters.L2;
tractor_w   = 2.5;
trailer_w   = 2.5;

% Camera window size
cam_w = 25;
cam_h = 18;

% Animation step
skip = 5;
num_steps = length(logs.x1);

for k = 1:skip:num_steps

    % Current poses
    x1 = logs.x1(k);
    y1 = logs.y1(k);
    p1 = logs.psi1(k);

    x2 = logs.x2(k);
    y2 = logs.y2(k);
    p2 = logs.psi2(k);

    % Update trailer trajectory
    set(h_traj, 'XData', logs.x2(1:k), 'YData', logs.y2(1:k));

    % Trailer box
    c2x = x2 + (trailer_len/2) * cos(p2);
    c2y = y2 + (trailer_len/2) * sin(p2);
    [Xt, Yt] = get_box_coords(c2x, c2y, p2, trailer_len, trailer_w);
    set(h_trailer, 'XData', Xt, 'YData', Yt);

    % Tractor box
    c1x = x1 + (tractor_len/2) * cos(p1);
    c1y = y1 + (tractor_len/2) * sin(p1);
    [Xc, Yc] = get_box_coords(c1x, c1y, p1, tractor_len, tractor_w);
    set(h_tractor, 'XData', Xc, 'YData', Yc);

    % Follow camera around trailer
    xlim([x2 - cam_w, x2 + cam_w]);
    ylim([y2 - cam_h, y2 + cam_h]);

    drawnow;
end


function [X, Y] = get_box_coords(cx, cy, psi, len, wid)
    hl = len/2;
    hw = wid/2;

    P = [ hl,  hw;
          hl, -hw;
         -hl, -hw;
         -hl,  hw ]';

    R = [cos(psi) -sin(psi);
         sin(psi)  cos(psi)];

    Pw = R * P + [cx; cy];
    X = Pw(1,:);
    Y = Pw(2,:);
end
