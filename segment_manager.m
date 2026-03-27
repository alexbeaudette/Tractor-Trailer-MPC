function [mode, label, switch_flag] = segment_manager(X0, x_r, y_r, theta_r, label_r, dir_r, prev_label, prev_idx)

% Segment manager block
%
% Inputs:
%   X0          : current state [X2; Y2; psi2; gamma]
%   prev_label  : previous path segment label
%   prev_idx    : previous switching flag
%   x_r         : path x coordinates
%   y_r         : path y coordinates
%   theta_r     : path heading
%   label_r     : path labels
%
% Outputs:
%   mode        : motion mode (+1 forward, -1 reverse)
%   label       : updated path segment label
%   switch_flag : updated switching flag (segment completion)

label = prev_label;
switch_flag = 0;
mode = 1;

% Current state components
X2 = X0(1);
Y2 = X0(2);

% Get indices of current segment
seg_inds = find(label_r == prev_label);

if isempty(seg_inds)
    switch_flag = prev_idx;
    label = prev_label;
else
    % Closest point on current segment using helper
    [~, ~, S0, ~] = distance_calc(X2, Y2, prev_label, x_r, y_r, theta_r, label_r);

    % End of current segment
    S_end = [x_r(seg_inds(end)); y_r(seg_inds(end))];
    d_end = norm(S0 - S_end);

    % Check if segment finished
    if d_end < 0.05
        switch_flag = 1;
    else
        switch_flag = 0;
    end

    switch_flag = max(switch_flag, prev_idx);

    % Advance segment if needed
    if switch_flag == 1
        label = prev_label + 1;
        switch_flag = 0;
    else
        label = prev_label;
    end
end

% Motion direction from active segment
seg_inds = find(label_r == label);

if isempty(seg_inds)
    mode = 1;
else
    mode = dir_r(seg_inds(1));
    if mode == 0
        mode = 1;
    end
end

end