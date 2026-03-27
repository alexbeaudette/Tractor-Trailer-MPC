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
s_r     = path(:,4);          %#ok<NASGU>
kappa_r = path(:,5);          %#ok<NASGU>
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

    %% Error calculation
    [errors, X_ref0, Y_ref0, theta_ref0, i0] = error_calculation( ...
        X, label, x_r, y_r, theta_r, label_r);

    % Debugging
    psi_err = wrapToPi(psi2 - theta_ref0);
    closest_dist = hypot(x2 - X_ref0, y2 - Y_ref0);
    %% Linearization and discretization
    [A, B, W] = truck_trailer_linearization(X, delta_prev, ux, L1, L1c, L2);
    [Ad, Bd, Wd] = truck_trailer_discretization(A, B, W, nu, nx, Ts);

    %% Predictive horizon
    [X_des, Y_des, theta_des] = predictive_horizon( ...
        X, Ad, Bd, Wd, delta_prev, label, x_r, y_r, theta_r, label_r, N);

    %% Desired hitch angle
    [gamma_ref, theta_h, s_rem, sT, iT, theta_goal] = solve_gamma_des( ...
        x2, y2, psi2, ux, i0, label, x_r, y_r, theta_r, label_r, dir_r, L1, Llook);

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

% Path tracking
figure('Color','k');
set(gca,'Color','k','XColor','w','YColor','w');
hold on; grid on; axis equal;
plot(x_r, y_r, 'w--', 'LineWidth', 1.5);
plot(logs.x2, logs.y2, 'w', 'LineWidth', 2);
xlabel('X [m]', 'Color','w');
ylabel('Y [m]', 'Color','w');
title('Path Tracking', 'Color','w');
legend('Reference Path','Trailer Path','TextColor','w','Location','best');

% Trailer heading vs reference
figure('Color','k');
set(gca,'Color','k','XColor','w','YColor','w');
hold on; grid on;
plot(t, rad2deg(logs.psi2), 'w', 'LineWidth', 1.8);
plot(t, rad2deg(logs.theta_ref0), 'w--', 'LineWidth', 1.5);
xlabel('Time [s]', 'Color','w');
ylabel('Angle [deg]', 'Color','w');
title('Trailer Heading Tracking', 'Color','w');
legend('\psi_2','\theta_{ref,0}','TextColor','w','Location','best');
xlim([t(1) t(end)]);

% Hitch angle vs reference
figure('Color','k');
set(gca,'Color','k','XColor','w','YColor','w');
hold on; grid on;
plot(t, rad2deg(logs.gamma), 'w', 'LineWidth', 1.8);
plot(t, rad2deg(logs.gamma_ref), 'w--', 'LineWidth', 1.5);
xlabel('Time [s]', 'Color','w');
ylabel('Angle [deg]', 'Color','w');
title('Hitch Angle Tracking', 'Color','w');
legend('\gamma','\gamma_{ref}','TextColor','w','Location','best');
xlim([t(1) t(end)]);

% Steering input
figure('Color','k');
set(gca,'Color','k','XColor','w','YColor','w');
hold on; grid on;
plot(t, rad2deg(logs.delta), 'w', 'LineWidth', 1.8);
yline(rad2deg(delta_umax), 'w--', 'LineWidth', 1.4);
yline(rad2deg(delta_umin), 'w:',  'LineWidth', 1.8);
xlabel('Time [s]', 'Color','w');
ylabel('Steering [deg]', 'Color','w');
title('Steering Input', 'Color','w');
legend('\delta','Upper Limit','Lower Limit','TextColor','w','Location','best');
xlim([t(1) t(end)]);

% Tracking error
figure('Color','k');
set(gca,'Color','k','XColor','w','YColor','w');
hold on; grid on;
plot(t, logs.errors, 'w', 'LineWidth', 1.8);
xlabel('Time [s]', 'Color','w');
ylabel('Error [m]', 'Color','w');
title('Tracking Error', 'Color','w');
xlim([t(1) t(end)]);

% Segment mode
figure('Color','k');
set(gca,'Color','k','XColor','w','YColor','w');
hold on; grid on;
plot(t, logs.mode, 'w', 'LineWidth', 1.8);
xlabel('Time [s]', 'Color','w');
ylabel('Mode', 'Color','w');
title('Segment Direction / Mode', 'Color','w');
xlim([t(1) t(end)]);

% Closest point and lookahead indices
figure('Color','k');
set(gca,'Color','k','XColor','w','YColor','w');
hold on; grid on;
plot(t, logs.i0, 'w', 'LineWidth', 1.8);
plot(t, logs.iT, 'w--', 'LineWidth', 1.5);
xlabel('Time [s]', 'Color','w');
ylabel('Path Index', 'Color','w');
title('Closest-Point and Lookahead Indices', 'Color','w');
legend('i0','iT','TextColor','w','Location','best');
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