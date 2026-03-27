function path = path_generation(path_type, ux, x_start, y_start, theta_path)

if nargin < 3, x_start = -13.0062; end
if nargin < 4, y_start = 0.0; end
if nargin < 5, theta_path = 0; end


% General settings
Nref = 8000;

switch lower(path_type)
    % Lane Merging
    case "merge"

        x_extent = 120;

        y_start  = 0.0;
        y_end    = 8.0;

        x_width  = 8;

        if ux >= 0
            x_center = +40;
        else
            x_center = -40;
        end

        x_r = linspace(-x_extent, +x_extent, Nref).';

        if ux < 0
            sigma = 0.5*(1 - tanh((x_r - x_center)/x_width));
        else
            sigma = 0.5*(1 + tanh((x_r - x_center)/x_width));
        end

        y_r = y_start + (y_end - y_start).*sigma;

        dx = gradient(x_r);
        dy = gradient(y_r);
        theta_r = unwrap(atan2(dy, dx));

        dir_raw = sign_nonzero(ux)*ones(numel(x_r), 1);
        label_raw = ones(numel(x_r), 1);

    % Straight line
    case "line"
        % Long straight line passing through (x_start, y_start)
        theta = deg2rad(theta_path); % line angle
        L_back = 80;
        L_fwd  = 180;
        % Creating linear range of distances
        s = linspace(-L_back, L_fwd, Nref).';
        % Projection onto x and y axes
        x_r = x_start + s*cos(theta);
        y_r = y_start + s*sin(theta);
        dx = gradient(x_r);
        dy = gradient(y_r);
        theta_r = unwrap(atan2(dy, dx));

        dir_raw = sign_nonzero(ux)*ones(numel(x_r), 1);
        label_raw = ones(numel(x_r), 1);

    % Circluar path
    case "circle"
        R = 50;
        t = linspace(0, 2*pi, Nref+1).';  % +1 to close then drop last
        t(end) = [];

        % Place the circle center so the point at angle pi is near start
        % angle pi gives x = -R, y = 0 relative to center
        cx = x_start;
        cy = y_start + R;

        x_r = cx + R*cos(t);
        y_r = cy + R*sin(t);

        % Rotate so index 1 is closest to start
        [x_r, y_r] = rotate_to_start(x_r, y_r, x_start, y_start);
        dx = gradient(x_r);
        dy = gradient(y_r);
        theta_r = unwrap(atan2(dy, dx));

        dir_raw = sign_nonzero(ux)*ones(numel(x_r), 1);
        label_raw = ones(numel(x_r), 1);
        
    % Figure 8 path
    case "figure8"
        A = 60;
        B = 30;
    
        t = linspace(0, 2*pi, Nref+1).';
        t(end) = [];
    
        place_at = "top";  % "top" or "bottom"
    
        if place_at == "top"
            xc = x_start - A/sqrt(2);
            yc = y_start - B;
        else
            xc = x_start - A/sqrt(2);
            yc = y_start + B;
        end
    
        x_r = xc + A*sin(t);
        y_r = yc + B*sin(2*t);

        dx = gradient(x_r);
        dy = gradient(y_r);
        theta_r = unwrap(atan2(dy, dx));

        dir_raw = sign_nonzero(ux)*ones(numel(x_r), 1);
        label_raw = ones(numel(x_r), 1);

    % Forward-Reverse parking path 
    % Path is continuous in s. You decide when to flip ux sign in your main loop.
    case "parkingfr"
        Nref = Nref*2;

        % ===== User-tunable geometry =====
        L1 = 50;              % length of first straight (m)
        R1 = 80;              % radius of arc 1 (m)
        phi1 = deg2rad(50);   % arc 1 angle (deg -> rad), small bend
    
        R2 = 80;               % radius of arc 2 (m)    
        L4 = 55;              % final vertical length downward (m)
    
        % Sampling
        N1 = floor(0.30*Nref);
        N2 = floor(0.20*Nref);
        N3 = floor(0.20*Nref);
        N4 = Nref - (N1+N2+N3);
    
        % ===== Segment 1: straight along +x =====
        x1 = linspace(0, L1, N1).';
        y1 = zeros(N1,1);
    
        % ===== Segment 2: arc 1, starts tangent to +x, turns upward =====
        % Use circle centered above the straight line so tangent at start is +x.
        % Param: theta from -pi/2 upward by phi1
        th2 = linspace(-pi/2, -pi/2 + phi1, N2).';
        cx1 = L1;
        cy1 = R1;
        x2 = cx1 + R1*cos(th2);
        y2 = cy1 + R1*sin(th2);
    
        % ===== Segment 3: arc 2, opposite hook =====
        P2 = [x2(end); y2(end)];
        psi2_end = th2(end) + pi/2;   % tangent direction at end of arc 1
        
        % (your existing center construction)
        nR = [sin(psi2_end); -cos(psi2_end)];
        C2 = P2 + R2*nR;
        
        % Angle of P2 around C2
        th3_start = atan2(P2(2)-C2(2), P2(1)-C2(1));
        
        % ---- enforce vertical tangent at end of arc 2 ----
        psi_end_des = -pi/2;          % vertical DOWN (use +pi/2 for vertical UP)
        
        % For CCW parameterization: psi = th + pi/2  => th_end_des = psi_end_des - pi/2
        th3_end_des = psi_end_des - pi/2;   % = -pi for vertical down
        
        % Choose phi2 so that th3_end = th3_start + phi2 hits th3_end_des (mod 2pi)
        phi2 = mod(th3_end_des - th3_start, 2*pi);
        
        % Optional: keep it the "shorter" arc (avoid huge sweeps)
        if phi2 > pi
            phi2 = phi2 - 2*pi;  % becomes negative (CW)
        end
        
        % Now generate th3 in the correct direction
        th3 = linspace(th3_start, th3_start + phi2, N3).';
        x3 = C2(1) + R2*cos(th3);
        y3 = C2(2) + R2*sin(th3);
    
        % ===== Segment 4: straight down (vertical) from end of arc 2 =====
        x4 = repmat(x3(end), N4, 1);
        y4 = linspace(y3(end), y3(end) - L4, N4).';
    
        % ===== Direction per segment =====
        dir1 =  ones(numel(x1),1);   % forward
        dir2 =  ones(numel(x2),1);   % forward
        dir3 = -ones(numel(x3),1);   % reverse
        dir4 = -ones(numel(x4),1);   % reverse

        label1 = 1*ones(numel(x1),1);
        label2 = 2*ones(numel(x2),1);
        label3 = 3*ones(numel(x3),1);
        label4 = 4*ones(numel(x4),1);

        % ===== Combine (avoid duplicate points at joins) =====
        x_raw = [x1;
                 x2(2:end);
                 x3(2:end);
                 x4(2:end)];
        y_raw = [y1;
                 y2(2:end);
                 y3(2:end);
                 y4(2:end)];

        dir_raw = [dir1;
                 dir2(2:end);
                 dir3(2:end);
                 dir4(2:end)];

        label_raw = [label1;
                 label2(2:end);
                 label3(2:end);
                 label4(2:end)];

        % Trim/pad to exactly Nref
        x_raw = x_raw(1:min(end,Nref));
        y_raw = y_raw(1:min(end,Nref));
        dir_raw = dir_raw(1:min(end,Nref));
        label_raw = label_raw(1:min(end,Nref));

        if numel(x_raw) < Nref
            x_raw(end+1:Nref,1) = x_raw(end);
            y_raw(end+1:Nref,1) = y_raw(end);
            dir_raw(end+1:Nref,1) = dir_raw(end);

        end
        
        if numel(x_raw) < Nref
            label_raw(end+1:Nref,1) = label_raw(end);
        end

        % ===== Rotate + translate to your requested start pose =====
        th = deg2rad(theta_path);
        x_r = x_start + (x_raw*cos(th) - y_raw*sin(th));
        y_r = y_start + (x_raw*sin(th) + y_raw*cos(th));

                
        i_sw = find(diff(dir_raw) ~= 0, 1, 'first');
        
        theta_r = zeros(size(x_r));
        
        % Forward portion
        dx1 = gradient(x_r(1:i_sw));
        dy1 = gradient(y_r(1:i_sw));
        theta_r(1:i_sw) = atan2(dy1, dx1);
        
        % Reverse portion
        dx2 = gradient(x_r(i_sw+1:end));
        dy2 = gradient(y_r(i_sw+1:end));
        theta_r(i_sw+1:end) = atan2(-dy2, -dx2);
        
        theta_r = unwrap(theta_r);

    otherwise
        error("Unknown path_type selected.");
end


% Helper calculations and path creation

dx  = gradient(x_r);
dy  = gradient(y_r);
ddx = gradient(dx);
ddy = gradient(dy);

kappa_r = (dx .* ddy - dy .* ddx) ./ max((dx.^2 + dy.^2).^(3/2), 1e-9);
if strcmpi(path_type, "parkingfr")
    kappa_r(i_sw-1:i_sw) = kappa_r(i_sw-2);
end

ds = hypot(diff(x_r), diff(y_r));
s_r = [0; cumsum(ds)];

path = [x_r, y_r, theta_r, s_r, kappa_r, dir_raw, label_raw];

% Plotting path
figure('Name','Path');
plot(x_r, y_r, '--','LineWidth', 2.2);
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
title('Path');

% % ---- Debug plot of individual segments ----
% figure('Name','Parking Segments'); hold on; grid on; axis equal;
% 
% plot(x1, y1, 'r', 'LineWidth', 2);      % Segment 1: straight
% plot(x2, y2, 'b', 'LineWidth', 2);      % Segment 2: arc 1
% plot(x3, y3, 'm', 'LineWidth', 2);      % Segment 3: arc 2
% plot(x4, y4, 'g', 'LineWidth', 2);      % Segment 4: vertical
% 
% legend('Straight 1','Arc 1','Arc 2','Straight 2');
% xlabel('X (m)');
% ylabel('Y (m)');
% title('Segment Debug View');

end

function [x2, y2] = rotate_to_start(x, y, xs, ys)
    dx = x - xs;
    dy = y - ys;
    [~, i0] = min(dx.*dx + dy.*dy);
    
    x2 = [x(i0:end); x(1:i0-1)];
    y2 = [y(i0:end); y(1:i0-1)];
end

function s = sign_nonzero(x)
    s = sign(x);
    if s == 0
        s = 1;
    end
end