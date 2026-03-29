%% MPC Script version - Rev1
% hit 'Run Section' for top section of this script to initialize parameters,
% path generation, and add paths to necessary functions
% clear; clc; close all;

script_path = mfilename('fullpath');
if isempty(script_path)
    script_dir = pwd;
else
    script_dir = fileparts(script_path);
end

qpoases_dir = fullfile(script_dir, 'qpOASES-slim');
qpoases_mex = fullfile(qpoases_dir, ['qpOASES.', mexext]);

if ~isfolder(qpoases_dir)
    error('qpOASES folder not found. Expected location: %s', qpoases_dir);
end

if ~isfile(qpoases_mex)
    error('qpOASES MEX file not found. Expected location: %s', qpoases_mex);
end

addpath(qpoases_dir)

%% Vehicle / simulation settings
ux = -1.0;                    % constant longitudinal speed [m/s]
Nsim = 8000;                   % number of simulation steps
save_debug_outputs = true;     % save logs and plots for offline diagnosis

%% Path definition
path_type = "merge";           % "merge", "line", "circle", "figure8", "parkingfr"
init_mode = "manual";        % "on_path" or "manual"

% Initial path anchoring
x_start = -13.006;
y_start = 1;
theta_path = -5;


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

if init_mode == "on_path"
    [X, i_init] = initialize_on_path_state(X, prev_label, x_r, y_r, theta_r, label_r);
    delta_prev = 0;
    prev_label = label_r(i_init);
    switch_flag = 0;
elseif init_mode ~= "manual"
    error('Unknown init_mode: %s', init_mode);
end

X_init = X;

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
logs.X_refT     = nan(1, Nsim);
logs.Y_refT     = nan(1, Nsim);
logs.theta_ref0 = nan(1, Nsim);
logs.errors     = nan(1, Nsim);

% Desired/reference quantities
logs.gamma_ref  = nan(1, Nsim);
logs.theta_h    = nan(1, Nsim);
logs.theta_goal = nan(1, Nsim);
logs.theta_pathT = nan(1, Nsim);
logs.heading_err_ref = nan(1, Nsim);
logs.ref_mode   = nan(1, Nsim);
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
logs.ux_sign = nan(1, Nsim);
logs.signed_cte = nan(1, Nsim);
logs.along_track_err = nan(1, Nsim);
logs.ex_ref0 = nan(1, Nsim);
logs.ey_ref0 = nan(1, Nsim);
logs.epsi_ref0 = nan(1, Nsim);
logs.X_des_1 = nan(1, Nsim);
logs.Y_des_1 = nan(1, Nsim);
logs.theta_des_1 = nan(1, Nsim);
logs.X_des_N = nan(1, Nsim);
logs.Y_des_N = nan(1, Nsim);
logs.theta_des_N = nan(1, Nsim);
logs.delta_plan_2 = nan(1, Nsim);
logs.delta_plan_5 = nan(1, Nsim);
logs.iN_ref = nan(1, Nsim);
logs.x2_pred = nan(N, Nsim);
logs.y2_pred = nan(N, Nsim);
logs.psi2_pred = nan(N, Nsim);
logs.gamma_pred = nan(N, Nsim);
logs.ex_pred_1 = nan(1, Nsim);
logs.ey_pred_1 = nan(1, Nsim);
logs.epsi_pred_1 = nan(1, Nsim);
logs.ex_pred_N = nan(1, Nsim);
logs.ey_pred_N = nan(1, Nsim);
logs.epsi_pred_N = nan(1, Nsim);

%% Simulation loop
for k = 1:Nsim

    % Reconstruct measurements from current state
    meas = get_measurements(X, ux, L1c, L2);

    x1    = meas.x1;
    y1    = meas.y1;
    x2    = meas.x2;
    y2    = meas.y2;
    psi1  = meas.psi1;
    psi2  = meas.psi2;
    gamma = meas.gamma;

    % Segment manager
    [mode, label, switch_flag] = segment_manager( ...
        X, x_r, y_r, theta_r, label_r, dir_r, prev_label, switch_flag);

    % Linearization and discretization
    [A, B, W, Ad, Bd, Wd] = truck_trailer_model( ...
        X, delta_prev, ux, L1, L1c, L2, nu, nx, Ts);

    % Reference generation
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
    theta_pathT = ref.theta_pathT;
    heading_err_ref = ref.heading_err_ref;
    ref_mode = ref.ref_mode;
    X_refT = ref.X_refT;
    Y_refT = ref.Y_refT;

    % Debugging
    psi_err = wrapToPi(psi2 - theta_ref0);
    closest_dist = hypot(x2 - X_ref0, y2 - Y_ref0);
    path_error_vec = [x2; y2] - [X_ref0; Y_ref0];
    path_tangent = [cos(theta_ref0); sin(theta_ref0)];
    path_normal = [-sin(theta_ref0); cos(theta_ref0)];
    signed_cte = path_normal.' * path_error_vec;
    along_track_err = path_tangent.' * path_error_vec;
    ex_ref0 = x2 - X_ref0;
    ey_ref0 = y2 - Y_ref0;
    epsi_ref0 = wrapToPi(psi2 - theta_ref0);

    % Direction-dependent Q
    if isempty(mode)
        mode = 1;
    end

    if mode < 0
        Q = Q_rev;
    else
        Q = Q_fwd;
    end

    % Build stacked reference vector
    X_ref = zeros(nx*N,1);
    for i = 1:N
        X_ref(nx*(i-1)+1:nx*i) = [X_des(i); Y_des(i); theta_des(i); gamma_ref];
    end

    % Solve optimization
    U = optimization(Ad, Bd, Wd, Q, R, P, N, delta_prev, X, X_ref, ...
                     umin, umax, delta_umin, delta_umax);

    u = reshape(U, nu, N);
    delta = u(:,1);

    % Plant propagation
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
    logs.X_refT(k)     = X_refT;
    logs.Y_refT(k)     = Y_refT;
    logs.theta_ref0(k) = theta_ref0;
    logs.errors(k)     = errors;

    logs.X_des(:,k)     = X_des;
    logs.Y_des(:,k)     = Y_des;
    logs.theta_des(:,k) = theta_des;

    logs.gamma_ref(k)  = gamma_ref;
    logs.theta_h(k)    = theta_h;
    logs.theta_goal(k) = theta_goal;
    logs.theta_pathT(k) = theta_pathT;
    logs.heading_err_ref(k) = heading_err_ref;
    logs.ref_mode(k)   = ref_mode;
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
    logs.ux_sign(k) = sign(ux);
    logs.signed_cte(k) = signed_cte;
    logs.along_track_err(k) = along_track_err;
    logs.ex_ref0(k) = ex_ref0;
    logs.ey_ref0(k) = ey_ref0;
    logs.epsi_ref0(k) = epsi_ref0;
    logs.X_des_1(k) = X_des(1);
    logs.Y_des_1(k) = Y_des(1);
    logs.theta_des_1(k) = theta_des(1);
    logs.X_des_N(k) = X_des(end);
    logs.Y_des_N(k) = Y_des(end);
    logs.theta_des_N(k) = theta_des(end);

    if N >= 2
        logs.delta_plan_2(k) = u(1,2);
    end

    if N >= 5
        logs.delta_plan_5(k) = u(1,5);
    end

    logs.iN_ref(k) = nearest_path_index(X_des(end), Y_des(end), x_r, y_r);

    % Update for next iteration
    X = X_next;
    delta_prev = delta;
    prev_label = label;
end

num_steps = find(isfinite(logs.x1), 1, 'last');
if isempty(num_steps)
    num_steps = 0;
end

logs = build_intent_debug_logs(logs, num_steps, x_r, y_r, umax);
snapshot_steps = logs.snapshot_steps;
intent_step = snapshot_steps.max_heading_error;
if ~isfinite(intent_step)
    intent_step = snapshot_steps.last_valid;
end
intent_bounds = compute_plot_bounds(x_r, y_r, logs.x2(1:max(num_steps,1)), logs.y2(1:max(num_steps,1)), 6);

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
h_umax = yline(rad2deg(umax), '--', 'Color', col_limit_hi, 'LineWidth', 1.4);
h_umin = yline(rad2deg(umin), ':', 'Color', col_limit_lo, 'LineWidth', 1.8);
xlabel('Time [s]', 'Color','w');
ylabel('Steering [deg]', 'Color','w');
title('Steering Input', 'Color','w');
legend([h_delta, h_umax, h_umin], {'\delta','Upper Limit','Lower Limit'}, 'TextColor','w','Location','best');
xlim([t(1) t(end)]);

% Current reference state errors
fig_curr_ref_err = figure('Color','k', 'Name', 'Current Reference State Errors');
tlo_curr = tiledlayout(fig_curr_ref_err, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tlo_curr, 'Current Reference State Errors', 'Color', 'w');

ax_ex = nexttile;
set(ax_ex, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
hold(ax_ex, 'on'); grid(ax_ex, 'on');
plot(ax_ex, t, logs.ex_ref0, 'Color', col_path, 'LineWidth', 1.8);
ylabel(ax_ex, 'x_2 - X_{ref,0} [m]', 'Color', 'w');
xlim(ax_ex, [t(1) t(end)]);

ax_ey = nexttile;
set(ax_ey, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
hold(ax_ey, 'on'); grid(ax_ey, 'on');
plot(ax_ey, t, logs.ey_ref0, 'Color', [1.00 0.45 0.25], 'LineWidth', 1.8);
ylabel(ax_ey, 'y_2 - Y_{ref,0} [m]', 'Color', 'w');
xlim(ax_ey, [t(1) t(end)]);

ax_epsi = nexttile;
set(ax_epsi, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
hold(ax_epsi, 'on'); grid(ax_epsi, 'on');
plot(ax_epsi, t, rad2deg(logs.epsi_ref0), 'Color', col_heading, 'LineWidth', 1.8);
xlabel(ax_epsi, 'Time [s]', 'Color', 'w');
ylabel(ax_epsi, '\psi_2 - \theta_{ref,0} [deg]', 'Color', 'w');
xlim(ax_epsi, [t(1) t(end)]);

% Predicted horizon tracking errors
fig_pred_err = figure('Color','k', 'Name', 'Predicted Horizon Tracking Errors');
tlo_pred = tiledlayout(fig_pred_err, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tlo_pred, 'Predicted vs Horizon Reference Errors', 'Color', 'w');

ax_pred_x = nexttile;
set(ax_pred_x, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
hold(ax_pred_x, 'on'); grid(ax_pred_x, 'on');
h_ex1 = plot(ax_pred_x, t, logs.ex_pred_1, 'Color', col_path, 'LineWidth', 1.8);
h_exN = plot(ax_pred_x, t, logs.ex_pred_N, '--', 'Color', [0.25 0.95 0.55], 'LineWidth', 1.5);
ylabel(ax_pred_x, 'x_{pred} - X_{des} [m]', 'Color', 'w');
legend(ax_pred_x, [h_ex1, h_exN], {'Step 1', 'Step N'}, 'TextColor', 'w', 'Location', 'best');
xlim(ax_pred_x, [t(1) t(end)]);

ax_pred_y = nexttile;
set(ax_pred_y, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
hold(ax_pred_y, 'on'); grid(ax_pred_y, 'on');
h_ey1 = plot(ax_pred_y, t, logs.ey_pred_1, 'Color', [1.00 0.45 0.25], 'LineWidth', 1.8);
h_eyN = plot(ax_pred_y, t, logs.ey_pred_N, '--', 'Color', [0.25 0.95 0.55], 'LineWidth', 1.5);
ylabel(ax_pred_y, 'y_{pred} - Y_{des} [m]', 'Color', 'w');
legend(ax_pred_y, [h_ey1, h_eyN], {'Step 1', 'Step N'}, 'TextColor', 'w', 'Location', 'best');
xlim(ax_pred_y, [t(1) t(end)]);

ax_pred_psi = nexttile;
set(ax_pred_psi, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
hold(ax_pred_psi, 'on'); grid(ax_pred_psi, 'on');
h_epsi1 = plot(ax_pred_psi, t, rad2deg(logs.epsi_pred_1), 'Color', col_heading, 'LineWidth', 1.8);
h_epsiN = plot(ax_pred_psi, t, rad2deg(logs.epsi_pred_N), '--', 'Color', [0.25 0.95 0.55], 'LineWidth', 1.5);
xlabel(ax_pred_psi, 'Time [s]', 'Color', 'w');
ylabel(ax_pred_psi, '\psi_{pred} - \theta_{des} [deg]', 'Color', 'w');
legend(ax_pred_psi, [h_epsi1, h_epsiN], {'Step 1', 'Step N'}, 'TextColor', 'w', 'Location', 'best');
xlim(ax_pred_psi, [t(1) t(end)]);

% Secondary path-frame errors
fig_path_metrics = figure('Color','k', 'Name', 'Secondary Path-Frame Errors');
set(gca,'Color','k','XColor','w','YColor','w');
hold on; grid on;
h_cte = plot(t, logs.signed_cte, 'Color', col_error, 'LineWidth', 1.8);
h_along = plot(t, logs.along_track_err, '--', 'Color', col_heading, 'LineWidth', 1.5);
xlabel('Time [s]', 'Color','w');
ylabel('Error [m]', 'Color','w');
title('Secondary Path-Frame Errors', 'Color','w');
legend([h_cte, h_along], {'Signed Cross-Track Error', 'Signed Along-Track Error'}, ...
    'TextColor','w','Location','best');
xlim([t(1) t(end)]);

% Commanded motion vs reference direction
figure('Color','k');
set(gca,'Color','k','XColor','w','YColor','w');
hold on; grid on;
h_ux_sign = plot(t, logs.ux_sign, 'Color', col_path, 'LineWidth', 1.8);
h_ref_mode = plot(t, logs.ref_mode, '--', 'Color', col_heading, 'LineWidth', 1.5);
h_mode = plot(t, logs.mode, ':', 'Color', col_mode, 'LineWidth', 1.6);
xlabel('Time [s]', 'Color','w');
ylabel('Direction Sign', 'Color','w');
title('Commanded Motion vs Reference Direction', 'Color','w');
legend([h_ux_sign, h_ref_mode, h_mode], {'sign(u_x)', 'ref\_mode', 'segment mode'}, ...
    'TextColor','w','Location','best');
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

% MPC intent dashboard
fig_dashboard = figure('Color','k', 'Name', 'MPC Intent Dashboard');
tlo = tiledlayout(fig_dashboard, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tlo, 'MPC Intent Dashboard', 'Color', 'w');

ax_heading = nexttile;
set(ax_heading, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
hold(ax_heading, 'on'); grid(ax_heading, 'on');
h_dash_psi2 = plot(ax_heading, t, rad2deg(logs.psi2), 'Color', col_path, 'LineWidth', 1.8);
h_dash_theta0 = plot(ax_heading, t, rad2deg(logs.theta_ref0), '--', 'Color', col_heading, 'LineWidth', 1.5);
h_dash_theta1 = plot(ax_heading, t, rad2deg(logs.theta_des_1), ':', 'Color', [1.00 0.35 0.35], 'LineWidth', 1.6);
h_dash_thetaN = plot(ax_heading, t, rad2deg(logs.theta_des_N), '-.', 'Color', [0.25 0.95 0.55], 'LineWidth', 1.5);
ylabel(ax_heading, 'Angle [deg]', 'Color', 'w');
legend(ax_heading, [h_dash_psi2, h_dash_theta0, h_dash_theta1, h_dash_thetaN], ...
    {'\psi_2', '\theta_{ref,0}', '\theta_{des}(1)', '\theta_{des}(N)'}, ...
    'TextColor', 'w', 'Location', 'best');
xlim(ax_heading, [t(1) t(end)]);

ax_track = nexttile;
set(ax_track, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
hold(ax_track, 'on'); grid(ax_track, 'on');
h_dash_cte = plot(ax_track, t, logs.signed_cte, 'Color', col_error, 'LineWidth', 1.8);
h_dash_along = plot(ax_track, t, logs.along_track_err, '--', 'Color', col_heading, 'LineWidth', 1.5);
title(ax_track, 'Secondary Path-Frame Errors', 'Color', 'w');
ylabel(ax_track, 'Error [m]', 'Color', 'w');
legend(ax_track, [h_dash_cte, h_dash_along], {'Signed CTE', 'Signed Along-Track Error'}, ...
    'TextColor', 'w', 'Location', 'best');
xlim(ax_track, [t(1) t(end)]);

ax_steer = nexttile;
set(ax_steer, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
hold(ax_steer, 'on'); grid(ax_steer, 'on');
h_dash_delta = plot(ax_steer, t, rad2deg(logs.delta), 'Color', col_path, 'LineWidth', 1.8);
h_dash_u2 = plot(ax_steer, t, rad2deg(logs.delta_plan_2), '--', 'Color', [1.00 0.35 0.35], 'LineWidth', 1.5);
h_dash_u5 = plot(ax_steer, t, rad2deg(logs.delta_plan_5), ':', 'Color', [0.25 0.95 0.55], 'LineWidth', 1.8);
yline(ax_steer, rad2deg(umax), '--', 'Color', col_limit_hi, 'LineWidth', 1.2);
yline(ax_steer, rad2deg(umin), ':', 'Color', col_limit_lo, 'LineWidth', 1.4);
ylabel(ax_steer, 'Steering [deg]', 'Color', 'w');
legend(ax_steer, [h_dash_delta, h_dash_u2, h_dash_u5], ...
    {'Applied \delta', 'Planned \delta(2)', 'Planned \delta(5)'}, ...
    'TextColor', 'w', 'Location', 'best');
xlim(ax_steer, [t(1) t(end)]);

ax_progress = nexttile;
set(ax_progress, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
hold(ax_progress, 'on'); grid(ax_progress, 'on');
h_dash_i0 = plot(ax_progress, t, logs.i0, 'Color', col_i0, 'LineWidth', 1.8);
h_dash_iT = plot(ax_progress, t, logs.iT, '--', 'Color', col_iT, 'LineWidth', 1.5);
h_dash_iN = plot(ax_progress, t, logs.iN_ref, ':', 'Color', [0.25 0.95 0.55], 'LineWidth', 1.8);
xlabel(ax_progress, 'Time [s]', 'Color', 'w');
ylabel(ax_progress, 'Path Index', 'Color', 'w');
legend(ax_progress, [h_dash_i0, h_dash_iT, h_dash_iN], ...
    {'i0', 'iT', 'i_{ref,end}'}, 'TextColor', 'w', 'Location', 'best');
xlim(ax_progress, [t(1) t(end)]);

% MPC intent spatial snapshot
fig_intent = figure('Color','k', 'Name', 'MPC Intent', 'Position', [120 120 1400 850]);
ax_intent = axes(fig_intent);
render_mpc_intent_snapshot(ax_intent, intent_step, logs, x_r, y_r, ...
    tractor_len, trailer_len, tractor_w, trailer_w, arrow_len, intent_bounds, Ts);

%% Animation for Tractor-Trailer Boxes

fig_anim = figure('Name','Tractor-Trailer Animation','Color','k');
ax_anim = axes(fig_anim);
hold(ax_anim, 'on');
grid(ax_anim, 'on');
axis(ax_anim, 'equal');
set(ax_anim, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');

h_ref_anim = plot(ax_anim, x_r, y_r, 'w--', 'LineWidth', 1.0);

xlabel(ax_anim, 'X (m)', 'Color', 'w');
ylabel(ax_anim, 'Y (m)', 'Color', 'w');
title(ax_anim, 'Animation: Vehicle Tracking Over Time', 'Color', 'w');

% Initialize graphic objects
h_traj = plot(ax_anim, nan, nan, 'w', 'LineWidth', 1.5);
h_closest = plot(ax_anim, nan, nan, 'o', ...
                 'MarkerSize', 6, ...
                 'MarkerFaceColor', [1.00 0.25 0.25], ...
                 'MarkerEdgeColor', [1.00 0.25 0.25]);
h_lookahead = plot(ax_anim, nan, nan, 'o', ...
                   'MarkerSize', 6, ...
                   'MarkerFaceColor', [0.20 0.95 1.00], ...
                   'MarkerEdgeColor', [0.20 0.95 1.00]);
h_ref_horizon = plot(ax_anim, nan, nan, '--o', ...
                     'Color', [1.00 0.45 0.25], ...
                     'LineWidth', 1.4, ...
                     'MarkerSize', 3, ...
                     'MarkerFaceColor', [1.00 0.45 0.25], ...
                     'MarkerEdgeColor', [1.00 0.45 0.25]);
h_pred_horizon = plot(ax_anim, nan, nan, '-o', ...
                      'Color', [0.25 0.95 0.55], ...
                      'LineWidth', 1.6, ...
                      'MarkerSize', 3, ...
                      'MarkerFaceColor', [0.25 0.95 0.55], ...
                      'MarkerEdgeColor', [0.25 0.95 0.55]);
h_closest_line = plot(ax_anim, nan, nan, ':', 'Color', [1.00 0.35 0.35], 'LineWidth', 1.4);
h_look_line = plot(ax_anim, nan, nan, '-', 'Color', [1.00 0.90 0.20], 'LineWidth', 1.5);
h_look_tangent = quiver(ax_anim, nan, nan, nan, nan, 0, ...
                        'Color', [1.00 0.10 1.00], 'LineWidth', 1.8, 'MaxHeadSize', 0.8);
h_status = text(ax_anim, 0.02, 0.98, '', ...
                'Units', 'normalized', ...
                'HorizontalAlignment', 'left', ...
                'VerticalAlignment', 'top', ...
                'Color', 'w', ...
                'FontName', 'Consolas', ...
                'FontSize', 10);

h_trailer = patch(ax_anim, 'XData', nan(1,4), 'YData', nan(1,4), ...
                  'FaceColor', [0.7 0.7 0.7], ...
                  'FaceAlpha', 0.35, ...
                  'EdgeColor', 'w', ...
                  'LineWidth', 1.2);

h_tractor = patch(ax_anim, 'XData', nan(1,4), 'YData', nan(1,4), ...
                  'FaceColor', [0.3 0.8 1.0], ...
                  'FaceAlpha', 0.35, ...
                  'EdgeColor', 'w', ...
                  'LineWidth', 1.2);

legend(ax_anim, [h_ref_anim, h_traj, h_ref_horizon, h_pred_horizon, ...
    h_closest, h_lookahead, h_closest_line, h_look_line, ...
    h_look_tangent, h_trailer, h_tractor], ...
    {'Reference Path', 'Trailer Path', 'Horizon Reference', 'Open-Loop Prediction', ...
     'Closest Point', 'Lookahead Point', 'To Closest Point', 'To Lookahead Point', ...
     'Lookahead Travel Dir', 'Trailer Body', 'Tractor Body'}, ...
    'TextColor', 'w', 'Location', 'bestoutside', 'AutoUpdate', 'off');

% Vehicle dimensions
tractor_len = parameters.L1;
trailer_len = parameters.L2;
tractor_w   = 2.5;
trailer_w   = 2.5;

% Camera window size
cam_w = 25;
cam_h = 18;

% Animation playback controls
animation_stride = 20;
animation_speed = 10.0;
arrow_len_anim = 2.0;

for k = 1:animation_stride:num_steps
    frame_start = tic;

    % Current poses
    x1 = logs.x1(k);
    y1 = logs.y1(k);
    p1 = logs.psi1(k);

    x2 = logs.x2(k);
    y2 = logs.y2(k);
    p2 = logs.psi2(k);
    xr0 = logs.X_ref0(k);
    yr0 = logs.Y_ref0(k);
    xrT = logs.X_refT(k);
    yrT = logs.Y_refT(k);
    thT = logs.theta_pathT(k);
    travel_sign = infer_travel_sign(logs, k);
    thT_travel = travel_heading(thT, travel_sign);
    x_ref_h = logs.X_des(:,k);
    y_ref_h = logs.Y_des(:,k);
    x_pred_h = logs.x2_pred(:,k);
    y_pred_h = logs.y2_pred(:,k);
    psi_err_deg = rad2deg(wrapToPi(p2 - logs.theta_ref0(k)));

    % Update trailer trajectory
    set(h_traj, 'XData', logs.x2(1:k), 'YData', logs.y2(1:k));
    set(h_closest, 'XData', xr0, 'YData', yr0);
    set(h_lookahead, 'XData', xrT, 'YData', yrT);
    set(h_ref_horizon, 'XData', x_ref_h, 'YData', y_ref_h);
    set(h_pred_horizon, 'XData', x_pred_h, 'YData', y_pred_h);
    set(h_closest_line, 'XData', [x2, xr0], 'YData', [y2, yr0]);
    set(h_look_line, 'XData', [x2, xrT], 'YData', [y2, yrT]);
    set(h_look_tangent, 'XData', xrT, 'YData', yrT, ...
        'UData', arrow_len_anim * cos(thT_travel), ...
        'VData', arrow_len_anim * sin(thT_travel));
    set(h_status, 'String', sprintf([ ...
        'k = %d (t = %.2f s)\n' ...
        'i0 = %d | iT = %d | iN = %d\n' ...
        'delta = %.2f deg | psi err = %.2f deg'], ...
        k, t(k), logs.i0(k), logs.iT(k), logs.iN_ref(k), ...
        rad2deg(logs.delta(k)), psi_err_deg));

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
    xlim(ax_anim, [x2 - cam_w, x2 + cam_w]);
    ylim(ax_anim, [y2 - cam_h, y2 + cam_h]);

    title(ax_anim, sprintf('Animation: Vehicle Tracking Over Time | k = %d', k), 'Color', 'w');
    drawnow;

    frame_duration = animation_stride * Ts / animation_speed;
    pause(max(0, frame_duration - toc(frame_start)));
end

%% Optional debug export
if save_debug_outputs
    run_stamp = datestr(now, 'yyyymmdd_HHMMSS');
    debug_dir = fullfile(script_dir, 'debug_runs', run_stamp);

    if ~exist(debug_dir, 'dir')
        mkdir(debug_dir);
    end

    run_info = struct();
    run_info.path_type = path_type;
    run_info.init_mode = init_mode;
    run_info.ux = ux;
    run_info.Nsim = Nsim;
    run_info.x_start = x_start;
    run_info.y_start = y_start;
    run_info.theta_path_deg = theta_path;
    run_info.X0 = X_init;
    run_info.save_time = datetime('now');
    run_info.snapshot_steps = snapshot_steps;
    run_info.intent_display_step = intent_step;

    save(fullfile(debug_dir, 'mpc_debug_run.mat'), ...
        'logs', 'path', 'run_info', 't');

    fig_handles = findall(groot, 'Type', 'figure');
    for iFig = 1:numel(fig_handles)
        fig = fig_handles(iFig);
        fig_name = get(fig, 'Name');

        if isempty(fig_name)
            fig_name = sprintf('figure_%02d', fig.Number);
        end

        fig_name = regexprep(fig_name, '[^\w\-]', '_');
        exportgraphics(fig, fullfile(debug_dir, [fig_name '.png']), 'Resolution', 150);
    end

    snapshot_exports = { ...
        'intent_start', snapshot_steps.first_valid; ...
        'intent_first_sat', snapshot_steps.first_saturation; ...
        'intent_max_heading_error', snapshot_steps.max_heading_error; ...
        'intent_final', snapshot_steps.last_valid};

    for iSnap = 1:size(snapshot_exports, 1)
        snap_name = snapshot_exports{iSnap, 1};
        snap_step = snapshot_exports{iSnap, 2};

        if ~isfinite(snap_step)
            continue
        end

        render_mpc_intent_snapshot(ax_intent, snap_step, logs, x_r, y_r, ...
            tractor_len, trailer_len, tractor_w, trailer_w, arrow_len, intent_bounds, Ts);
        exportgraphics(fig_intent, fullfile(debug_dir, [snap_name '.png']), 'Resolution', 150);
    end

    render_mpc_intent_snapshot(ax_intent, intent_step, logs, x_r, y_r, ...
        tractor_len, trailer_len, tractor_w, trailer_w, arrow_len, intent_bounds, Ts);

    fprintf('Saved debug outputs to: %s\n', debug_dir);
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

function [X_init, i_init] = initialize_on_path_state(X_seed, start_label, x_r, y_r, theta_r, label_r)
    idx = find(label_r == start_label);
    if isempty(idx)
        idx = (1:numel(x_r)).';
    end

    distances = hypot(x_r(idx) - X_seed(1), y_r(idx) - X_seed(2));
    [~, local_idx] = min(distances);
    i_init = idx(local_idx);

    X_init = X_seed;
    X_init(1) = x_r(i_init);
    X_init(2) = y_r(i_init);
    X_init(3) = wrapToPi(theta_r(i_init));
    X_init(4) = 0;
end

function logs = build_intent_debug_logs(logs, num_steps, x_r, y_r, umax)
    if num_steps <= 0
        logs.snapshot_steps = struct( ...
            'first_valid', nan, ...
            'first_saturation', nan, ...
            'max_heading_error', nan, ...
            'last_valid', nan);
        return
    end

    N = size(logs.u, 2);

    for k = 1:num_steps
        Xk = logs.X(:,k);
        Ad = logs.Ad(:,:,k);
        Bd = logs.Bd(:,:,k);
        Wd = logs.Wd(:,k);

        if any(~isfinite(Xk)) || any(~isfinite(Ad(:))) || any(~isfinite(Bd(:))) || any(~isfinite(Wd))
            continue
        end

        Xpred = Xk;
        for j = 1:N
            uj = logs.u(:,j,k);
            if any(~isfinite(uj))
                break
            end

            Xpred = Ad * Xpred + Bd * uj + Wd;
            logs.x2_pred(j,k) = Xpred(1);
            logs.y2_pred(j,k) = Xpred(2);
            logs.psi2_pred(j,k) = wrapToPi(Xpred(3));
            logs.gamma_pred(j,k) = wrapToPi(Xpred(4));
        end

        if isnan(logs.iN_ref(k)) && isfinite(logs.X_des_N(k)) && isfinite(logs.Y_des_N(k))
            logs.iN_ref(k) = nearest_path_index(logs.X_des_N(k), logs.Y_des_N(k), x_r, y_r);
        end

        if isfinite(logs.x2_pred(1,k)) && isfinite(logs.X_des(1,k)) && ...
                isfinite(logs.y2_pred(1,k)) && isfinite(logs.Y_des(1,k)) && ...
                isfinite(logs.psi2_pred(1,k)) && isfinite(logs.theta_des(1,k))
            logs.ex_pred_1(k) = logs.x2_pred(1,k) - logs.X_des(1,k);
            logs.ey_pred_1(k) = logs.y2_pred(1,k) - logs.Y_des(1,k);
            logs.epsi_pred_1(k) = wrapToPi(logs.psi2_pred(1,k) - logs.theta_des(1,k));
        end

        if isfinite(logs.x2_pred(N,k)) && isfinite(logs.X_des(N,k)) && ...
                isfinite(logs.y2_pred(N,k)) && isfinite(logs.Y_des(N,k)) && ...
                isfinite(logs.psi2_pred(N,k)) && isfinite(logs.theta_des(N,k))
            logs.ex_pred_N(k) = logs.x2_pred(N,k) - logs.X_des(N,k);
            logs.ey_pred_N(k) = logs.y2_pred(N,k) - logs.Y_des(N,k);
            logs.epsi_pred_N(k) = wrapToPi(logs.psi2_pred(N,k) - logs.theta_des(N,k));
        end
    end

    valid_idx = find(isfinite(logs.x2(1:num_steps)));
    heading_err = abs(wrapToPi(logs.psi2 - logs.theta_ref0));
    heading_err(~isfinite(heading_err)) = -inf;

    sat_tol = max(1e-6, 0.01 * abs(umax));
    sat_candidates = find(isfinite(logs.delta(1:num_steps)) & abs(abs(logs.delta(1:num_steps)) - abs(umax)) <= sat_tol, 1, 'first');

    first_valid = nan;
    last_valid = nan;
    max_heading_error = nan;

    if ~isempty(valid_idx)
        first_valid = valid_idx(1);
        last_valid = valid_idx(end);

        [~, local_idx] = max(heading_err(valid_idx));
        max_heading_error = valid_idx(local_idx);
    end

    if isempty(sat_candidates)
        first_saturation = nan;
    else
        first_saturation = sat_candidates;
    end

    logs.snapshot_steps = struct( ...
        'first_valid', first_valid, ...
        'first_saturation', first_saturation, ...
        'max_heading_error', max_heading_error, ...
        'last_valid', last_valid);
end

function idx = nearest_path_index(x_point, y_point, x_r, y_r)
    if ~isfinite(x_point) || ~isfinite(y_point)
        idx = nan;
        return
    end

    distances = hypot(x_r - x_point, y_r - y_point);
    [~, idx] = min(distances);
end

function bounds = compute_plot_bounds(x_ref, y_ref, x_act, y_act, margin)
    x_all = [x_ref(:); x_act(:)];
    y_all = [y_ref(:); y_act(:)];

    x_all = x_all(isfinite(x_all));
    y_all = y_all(isfinite(y_all));

    if isempty(x_all)
        x_all = 0;
    end

    if isempty(y_all)
        y_all = 0;
    end

    bounds = struct();
    bounds.x = [min(x_all) - margin, max(x_all) + margin];
    bounds.y = [min(y_all) - margin, max(y_all) + margin];
end

function render_mpc_intent_snapshot(ax, k, logs, x_r, y_r, tractor_len, trailer_len, tractor_w, trailer_w, arrow_len, bounds, Ts)
    cla(ax);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    set(ax, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');

    if ~isfinite(k) || k < 1 || k > numel(logs.x2) || ~isfinite(logs.x2(k))
        text(ax, 0.5, 0.5, 'No valid MPC intent snapshot available', ...
            'Units', 'normalized', ...
            'HorizontalAlignment', 'center', ...
            'Color', 'w');
        xlim(ax, bounds.x);
        ylim(ax, bounds.y);
        return
    end

    x1 = logs.x1(k);
    y1 = logs.y1(k);
    psi1 = logs.psi1(k);
    x2 = logs.x2(k);
    y2 = logs.y2(k);
    psi2 = logs.psi2(k);
    xr0 = logs.X_ref0(k);
    yr0 = logs.Y_ref0(k);
    xrT = logs.X_refT(k);
    yrT = logs.Y_refT(k);
    th0 = logs.theta_ref0(k);
    thT = logs.theta_pathT(k);
    travel_sign = infer_travel_sign(logs, k);
    th0_travel = travel_heading(th0, travel_sign);
    thT_travel = travel_heading(thT, travel_sign);

    h_ref = plot(ax, x_r, y_r, '--', 'Color', [0.70 0.70 0.70], 'LineWidth', 1.4);
    h_actual = plot(ax, logs.x2(1:k), logs.y2(1:k), 'Color', [0.20 0.80 1.00], 'LineWidth', 2.0);
    h_ref_h = plot(ax, logs.X_des(:,k), logs.Y_des(:,k), '--o', ...
        'Color', [1.00 0.45 0.25], 'LineWidth', 1.5, 'MarkerSize', 3, ...
        'MarkerFaceColor', [1.00 0.45 0.25], 'MarkerEdgeColor', [1.00 0.45 0.25]);
    h_pred_h = plot(ax, logs.x2_pred(:,k), logs.y2_pred(:,k), '-o', ...
        'Color', [0.25 0.95 0.55], 'LineWidth', 1.6, 'MarkerSize', 3, ...
        'MarkerFaceColor', [0.25 0.95 0.55], 'MarkerEdgeColor', [0.25 0.95 0.55]);
    h_current = plot(ax, x2, y2, 'o', 'MarkerSize', 7, ...
        'MarkerFaceColor', [1 1 1], 'MarkerEdgeColor', [1 1 1]);
    h_closest = plot(ax, xr0, yr0, 'o', 'MarkerSize', 6, ...
        'MarkerFaceColor', [1.00 0.25 0.25], 'MarkerEdgeColor', [1.00 0.25 0.25]);
    h_look = plot(ax, xrT, yrT, 'o', 'MarkerSize', 6, ...
        'MarkerFaceColor', [0.20 0.95 1.00], 'MarkerEdgeColor', [0.20 0.95 1.00]);
    h_closest_line = plot(ax, [x2, xr0], [y2, yr0], ':', 'Color', [1.00 0.35 0.35], 'LineWidth', 1.4);
    h_look_line = plot(ax, [x2, xrT], [y2, yrT], '-', 'Color', [1.00 0.90 0.20], 'LineWidth', 1.5);
    h_tangent0 = quiver(ax, xr0, yr0, arrow_len * cos(th0_travel), arrow_len * sin(th0_travel), 0, ...
        'Color', [1.00 0.10 1.00], 'LineWidth', 1.6, 'MaxHeadSize', 0.8);
    h_tangentT = quiver(ax, xrT, yrT, arrow_len * cos(thT_travel), arrow_len * sin(thT_travel), 0, ...
        'Color', [0.25 0.95 0.55], 'LineWidth', 1.6, 'MaxHeadSize', 0.8);

    c2x = x2 + (trailer_len/2) * cos(psi2);
    c2y = y2 + (trailer_len/2) * sin(psi2);
    [Xt, Yt] = get_box_coords(c2x, c2y, psi2, trailer_len, trailer_w);
    h_trailer = patch(ax, 'XData', Xt, 'YData', Yt, ...
        'FaceColor', [0.75 0.75 0.75], 'FaceAlpha', 0.18, ...
        'EdgeColor', [0.95 0.95 0.95], 'LineWidth', 1.5);

    c1x = x1 + (tractor_len/2) * cos(psi1);
    c1y = y1 + (tractor_len/2) * sin(psi1);
    [Xc, Yc] = get_box_coords(c1x, c1y, psi1, tractor_len, tractor_w);
    h_tractor = patch(ax, 'XData', Xc, 'YData', Yc, ...
        'FaceColor', [0.20 0.80 1.00], 'FaceAlpha', 0.20, ...
        'EdgeColor', [0.70 0.95 1.00], 'LineWidth', 1.4);

    heading_err_deg = rad2deg(wrapToPi(psi2 - th0));
    t_snapshot = (k - 1) * Ts;
    title(ax, sprintf(['MPC Intent | k = %d, t = %.2f s | ' ...
        '\\delta = %.2f deg | \\psi err = %.2f deg'], ...
        k, t_snapshot, ...
        rad2deg(logs.delta(k)), heading_err_deg), 'Color', 'w');
    xlabel(ax, 'X (m)', 'Color', 'w');
    ylabel(ax, 'Y (m)', 'Color', 'w');

    info_text = sprintf([ ...
        'i0 = %d\n' ...
        'iT = %d\n' ...
        'iN = %d\n' ...
        'ref mode = %+d\n' ...
        'signed CTE = %.3f m\n' ...
        'along-track = %.3f m'], ...
        logs.i0(k), logs.iT(k), logs.iN_ref(k), travel_sign, ...
        logs.signed_cte(k), logs.along_track_err(k));
    text(ax, 0.02, 0.98, info_text, ...
        'Units', 'normalized', ...
        'HorizontalAlignment', 'left', ...
        'VerticalAlignment', 'top', ...
        'Color', 'w', ...
        'FontName', 'Consolas', ...
        'FontSize', 10);

    legend(ax, [h_ref, h_actual, h_ref_h, h_pred_h, h_current, h_closest, h_look, ...
        h_closest_line, h_look_line, h_tangent0, h_tangentT, h_trailer, h_tractor], ...
        {'Reference Path', 'Trailer Path', 'Horizon Reference', 'Open-Loop Prediction', ...
         'Current Trailer Axle', 'Closest Point', 'Lookahead Point', ...
         'To Closest Point', 'To Lookahead Point', 'Closest Travel Dir', ...
         'Lookahead Travel Dir', 'Trailer Body', 'Tractor Body'}, ...
        'TextColor', 'w', 'Location', 'bestoutside');

    snapshot_bounds = compute_snapshot_plot_bounds(logs, k, xr0, yr0, xrT, yrT, 4, 6);
    xlim(ax, snapshot_bounds.x);
    ylim(ax, snapshot_bounds.y);
end

function travel_sign = infer_travel_sign(logs, k)
    candidates = [logs.ref_mode(k), logs.mode(k), sign(logs.ux(k))];
    travel_sign = 1;
    for i = 1:numel(candidates)
        candidate = candidates(i);
        if isfinite(candidate) && candidate ~= 0
            travel_sign = sign(candidate);
            return
        end
    end
end

function theta_travel = travel_heading(theta_path, travel_sign)
    theta_travel = wrapToPi(theta_path);
    if isfinite(travel_sign) && travel_sign < 0
        theta_travel = wrapToPi(theta_travel + pi);
    end
end

function bounds = compute_snapshot_plot_bounds(logs, k, xr0, yr0, xrT, yrT, margin, min_half_height)
    x_all = [logs.x2(k); logs.x1(k); xr0; xrT; logs.X_des(:,k); logs.x2_pred(:,k)];
    y_all = [logs.y2(k); logs.y1(k); yr0; yrT; logs.Y_des(:,k); logs.y2_pred(:,k)];

    x_all = x_all(isfinite(x_all));
    y_all = y_all(isfinite(y_all));

    if isempty(x_all)
        x_all = 0;
    end

    if isempty(y_all)
        y_all = 0;
    end

    x_center = 0.5 * (min(x_all) + max(x_all));
    y_center = 0.5 * (min(y_all) + max(y_all));

    half_width = max(0.5 * (max(x_all) - min(x_all)) + margin, 8);
    half_height = max(0.5 * (max(y_all) - min(y_all)) + margin, min_half_height);

    bounds = struct();
    bounds.x = [x_center - half_width, x_center + half_width];
    bounds.y = [y_center - half_height, y_center + half_height];
end
