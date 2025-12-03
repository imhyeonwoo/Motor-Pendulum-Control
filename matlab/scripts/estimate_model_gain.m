% estimate_model_gain.m
% Estimate motor gain (K) and time constant (tau) from step-response data.
% Procedure:
%   1) find peak time (Tp) and percent overshoot (%OS) from the output
%   2) use the controller transfer function (P control assumed)
%   3) zeta = -ln(%OS/100) / sqrt(pi^2 + ln^2(%OS/100))
%   4) omega_d = pi / Tp
%   5) omega_n = omega_d / sqrt(1 - zeta^2)
%   6) tau     = 1 / (2 * zeta * omega_n)
%   7) K       = omega_n^2 * tau / Kp   (from tau*s^2 + s + K*Kp = 0)

clear; close all; clc;

%% Settings
data_dir = find_pid_folder();

% Use one or two P-control datasets inside the 4*data/PID folder.
% You can change this list to try other files in the same folder.
files_to_use = {'p2.mat', 'p0.366.mat'};

% Optional: limit the analysis window [t_start, t_end] in seconds.
analysis_window = [];  % leave empty to use full data range

%% Run estimation
estimates = struct([]);

for k = 1:numel(files_to_use)
    fname = files_to_use{k};
    fpath = fullfile(data_dir, fname);

    if ~isfile(fpath)
        warning('File not found: %s (skipping)', fpath);
        continue;
    end

    raw = load(fpath);
    [time, ref, out, Kp, Ki, Kd] = extract_signals(raw, fname);

    if abs(Ki) > 1e-9 || abs(Kd) > 1e-9
        warning('Skipping %s because Ki or Kd is non-zero. The requested formulas assume P control.', fname);
        continue;
    end

    if ~isempty(analysis_window)
        idx = (time >= analysis_window(1)) & (time <= analysis_window(2));
        time = time(idx); ref = ref(idx); out = out(idx);
    end

    metrics = step_metrics(time, ref, out);
    if ~metrics.valid
        warning('Skipping %s (unable to compute Tp / OS).', fname);
        continue;
    end

    % Parameter estimation (steps 3-7)
    zeta     = metrics.zeta;
    omega_d  = metrics.omega_d;
    omega_n  = metrics.omega_n;
    tau_hat  = metrics.tau;
    K_hat    = (omega_n^2 * tau_hat) / Kp;

    fprintf('\n[%s]\n', fname);
    fprintf('  Controller: Kp=%.4g, Ki=%.4g, Kd=%.4g\n', Kp, Ki, Kd);
    fprintf('  Peak time (Tp)      : %.4f s\n', metrics.Tp);
    fprintf('  Overshoot (%%OS)     : %.2f %%\n', metrics.OS);
    fprintf('  zeta                : %.4f\n', zeta);
    fprintf('  omega_d             : %.4f rad/s\n', omega_d);
    fprintf('  omega_n             : %.4f rad/s\n', omega_n);
    fprintf('  tau (estimated)     : %.5f s\n', tau_hat);
    fprintf('  K   (estimated)     : %.5f\n', K_hat);

    idx_est = numel(estimates) + 1;
    estimates(idx_est) = struct( ... %#ok<SAGROW>
        'file', fname, ...
        'Tp', metrics.Tp, ...
        'OS', metrics.OS, ...
        'zeta', zeta, ...
        'omega_d', omega_d, ...
        'omega_n', omega_n, ...
        'tau', tau_hat, ...
        'K', K_hat, ...
        'Kp', Kp, 'Ki', Ki, 'Kd', Kd, ...
        'time', time, 'ref', ref, 'out', out); %#ok<AGROW>

    % Quick simulation to check fit against the measured reference/output.
    s = tf('s');
    G_hat = K_hat / (s * (tau_hat * s + 1));
    C     = Kp;  % P controller only
    T_hat = feedback(C * G_hat, 1);
    out_hat = lsim(T_hat, ref, time);

    figure('Name', sprintf('Estimated model: %s', fname), 'Color', 'w', ...
           'Position', [100 100 950 500]);
    plot(time, ref, 'k--', 'LineWidth', 1.0); hold on;
    plot(time, out, 'b', 'LineWidth', 1.2);
    plot(time, out_hat, 'r', 'LineWidth', 1.2);
    grid on;
    xlabel('Time [s]');
    ylabel('Angle [deg]');
    legend('Reference', 'Experiment', 'Sim w/ estimated K, \\tau', 'Location', 'best');
    title(sprintf('Step response fit (%s)', fname));
end

if isempty(estimates)
    warning('No estimates were produced. Check the chosen files or data content.');
else
    fprintf('\nSummary (P cases only)\n');
    fprintf('File\t\tTp[s]\tOS[%%]\tzeta\tomega_n[rad/s]\ttau[s]\tK\n');
    for k = 1:numel(estimates)
        e = estimates(k);
        fprintf('%s\t%.4f\t%.2f\t%.4f\t%.4f\t%.5f\t%.5f\n', ...
            e.file, e.Tp, e.OS, e.zeta, e.omega_n, e.tau, e.K);
    end
end

%% Helper functions
function data_dir = find_pid_folder()
    % Look for a folder that matches the pattern 4*data/PID by walking up a few levels.
    search_root = pwd;
    for i = 1:4
        candidates = dir(fullfile(search_root, '4*data'));
        for c = candidates'
            if ~c.isdir
                continue;
            end
            maybe = fullfile(search_root, c.name, 'PID');
            if isfolder(maybe)
                data_dir = maybe;
                return;
            end
        end
        parent = fileparts(search_root);
        if isempty(parent) || strcmp(parent, search_root)
            break;
        end
        search_root = parent;
    end
    error('Could not locate the PID data folder. Update data_dir manually.');
end

function [time, ref, out, Kp, Ki, Kd] = extract_signals(S, fname)
    time = ensure_col(get_field(S, {'time', 'Time', 't'}, 'time', fname));
    ref  = ensure_col(get_field(S, {'signal_generated', 'signal', 'Signal'}, 'reference', fname));
    out  = ensure_col(get_field(S, {'encoder', 'Encoder', 'theta'}, 'encoder', fname));

    Kp = mean(get_field(S, {'Kp_simout', 'Kp'}, 'Kp', fname));
    Ki = mean(get_field(S, {'Ki_simout', 'Ki'}, 'Ki', fname));
    Kd = mean(get_field(S, {'Kd_simout', 'Kd'}, 'Kd', fname));

    n = min([numel(time), numel(ref), numel(out)]);
    time = time(1:n); ref = ref(1:n); out = out(1:n);
end

function value = get_field(S, names, label, fname)
    for i = 1:numel(names)
        if isfield(S, names{i})
            value = S.(names{i});
            return;
        end
    end
    error('Missing "%s" field in %s.', label, fname);
end

function v = ensure_col(x)
    v = double(x(:));
end

function metrics = step_metrics(t, r, y)
    metrics = struct('valid', false, 'Tp', NaN, 'OS', NaN, ...
        'zeta', NaN, 'omega_d', NaN, 'omega_n', NaN, 'tau', NaN);

    if numel(t) < 10 || any(diff(t) <= 0)
        return;
    end

    % Find step edges in the reference (square-wave data exist; use first edge).
    edges = find(abs(diff(r)) > 1e-6);
    if isempty(edges)
        return;
    end

    idx_start = edges(1) + 1;   % first sample after the step
    if numel(edges) >= 2
        idx_stop = edges(2) + 1; % up to the next edge
    else
        idx_stop = numel(t);     % or to the end if only one edge
    end

    seg_idx = idx_start:idx_stop;
    if numel(seg_idx) < 10
        return;
    end

    % Baseline just before the step.
    pre_n = min(idx_start-1, max(10, round(0.05 * numel(t))));
    pre_range = (idx_start-pre_n):(idx_start-1);
    r0 = mean(r(pre_range));
    y0 = mean(y(pre_range));

    % Post-step segment (until next edge).
    t_seg = t(seg_idx);
    r_seg = r(seg_idx);
    y_seg = y(seg_idx);

    tail_n = max(10, round(0.1 * numel(r_seg)));
    r1 = mean(r_seg(end-tail_n+1:end));
    r_step = r1 - r0;
    if abs(r_step) < 1e-6
        return;
    end

    step_sign = sign(r_step);
    if step_sign >= 0
        [y_peak, idx_peak] = max(y_seg);
    else
        [y_peak, idx_peak] = min(y_seg);
    end

    Tp = t_seg(idx_peak) - t_seg(1); % time-to-peak from step instant

    % Percent overshoot relative to commanded step magnitude.
    y_peak_rel = y_peak - y0;
    OS = 100 * abs(y_peak_rel - r_step) / abs(r_step);

    if Tp <= 0 || OS <= 0
        return;
    end

    zeta = -log(OS/100) / sqrt(pi^2 + log(OS/100)^2);
    if zeta >= 1
        return;  % overdamped -> no peak; formula invalid
    end

    omega_d = pi / Tp;
    omega_n = omega_d / sqrt(1 - zeta^2);
    tau     = 1 / (2 * zeta * omega_n);

    metrics.valid   = isfinite(omega_n) && isfinite(tau);
    metrics.Tp      = Tp;
    metrics.OS      = OS;
    metrics.zeta    = zeta;
    metrics.omega_d = omega_d;
    metrics.omega_n = omega_n;
    metrics.tau     = tau;
end
