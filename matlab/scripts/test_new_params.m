%% test_new_params.m
% 새로 식별한 모터 파라미터를 P / PD 실험 데이터에 적용해 비교

clear; close all; clc;

%% 1. 모터 파라미터 설정
% (1) 기본 파라미터 (제공값)
K_nom   = 21.9;
tau_nom = 0.15;

% (2) 새로 식별한 파라미터 (estimate_model_gain.m 결과)
K_new   = 20.73;
tau_new = 0.124;   % 필요하면 최신 값으로 수정

%% 2. 데이터 폴더 및 실험 케이스 정의
base_dir = 'C:\Users\User\Desktop\Git\Motor-Pendulum-Control\4조 data\PID';

cases = struct([]);

% Case 1: P 제어 (Kp = 2)
cases(1).file = 'p2.mat';
cases(1).name = 'P control (Kp = 2)';
cases(1).type = 'P';
cases(1).Kp   = 2;
cases(1).Kd   = 0;

% Case 2: PD 제어 (Kp = 1, Kd = 0.05)
cases(2).file = 'p1d0.05.mat';
cases(2).name = 'PD control (Kp = 1, Kd = 0.05)';
cases(2).type = 'PD';
cases(2).Kp   = 1;
cases(2).Kd   = 0.05;

%% 3. 각 실험 케이스에 대해 비교 수행
for i = 1:numel(cases)
    c = cases(i);

    % --- 데이터 로드 ---
    data_path = fullfile(base_dir, c.file);
    S = load(data_path);

    time    = S.time(:);
    encoder = S.encoder(:);

    if isfield(S, 'signal_generated')
        signal = S.signal_generated(:);
    elseif isfield(S, 'signal')
        signal = S.signal(:);
    else
        error('signal / signal_generated 필드를 찾을 수 없습니다 (%s).', c.file);
    end

    % 원본 그대로 사용 (offset 안 빼고)
    t     = time;
    r     = signal;
    y_exp = encoder;

    % --- 기존 / 새 파라미터로 시뮬레이션 ---
    y_nom = simulate_motor_response(t, r, c, K_nom, tau_nom);
    y_new = simulate_motor_response(t, r, c, K_new, tau_new);

    % --- RMSE 계산 (전체 구간 기준) ---
    rmse_nom = sqrt(mean((y_nom - y_exp).^2));
    rmse_new = sqrt(mean((y_new - y_exp).^2));

    fprintf('\n=== %s ===\n', c.name);
    fprintf('  RMSE (nominal K=%.1f, tau=%.3f) = %.3f deg\n', ...
            K_nom, tau_nom, rmse_nom);
    fprintf('  RMSE (new     K=%.2f, tau=%.3f) = %.3f deg\n', ...
            K_new, tau_new, rmse_new);

    % --- 플롯 ---
    figure('Name', c.name, 'Position', [100 100 950 600]);

    % (a) 전체 응답
    subplot(2,1,1);
    plot(t, r,     'k--', 'LineWidth', 1.0); hold on;
    plot(t, y_exp, 'b',   'LineWidth', 1.2);
    plot(t, y_nom, 'r--', 'LineWidth', 1.2);
    plot(t, y_new, 'm',   'LineWidth', 1.2);
    grid on;
    xlabel('Time [s]');
    ylabel('Angle [deg]');
    title(sprintf('%s: Response Comparison (nominal vs new motor model)', c.name));
    legend('Reference (Input)', ...
           'Experiment (Encoder)', ...
           sprintf('Simulation (K=%.1f, \\tau=%.3f)', K_nom, tau_nom), ...
           sprintf('Simulation (K=%.2f, \\tau=%.3f)', K_new, tau_new), ...
           'Location','Best');

    % (b) 과도응답 확대 (앞쪽 0~5초 정도, 필요하면 수정)
    t_zoom_start = 0.0;
    t_zoom_end   = 5.0;
    idx_zoom = (t >= t_zoom_start) & (t <= t_zoom_end);

    subplot(2,1,2);
    plot(t(idx_zoom), r(idx_zoom),     'k--', 'LineWidth', 1.0); hold on;
    plot(t(idx_zoom), y_exp(idx_zoom), 'b',   'LineWidth', 1.2);
    plot(t(idx_zoom), y_nom(idx_zoom), 'r--','LineWidth', 1.2);
    plot(t(idx_zoom), y_new(idx_zoom), 'm',  'LineWidth', 1.2);
    grid on;
    xlabel('Time [s]');
    ylabel('Angle [deg]');
    title('Zoomed View (Transient Response)');
    legend('Reference (Input)', ...
           'Experiment (Encoder)', ...
           sprintf('Simulation (K=%.1f, \\tau=%.3f)', K_nom, tau_nom), ...
           sprintf('Simulation (K=%.2f, \\tau=%.3f)', K_new, tau_new), ...
           'Location','Best');
end

%% ====== 로컬 함수 ======
function y = simulate_motor_response(t, r, c, K, tau)
    % 모터 + 제어기 전달함수로 닫힌루프 응답 시뮬레이션
    s = tf('s');
    G = K / (s*(tau*s + 1));   % 모터 모델

    switch c.type
        case 'P'
            C = c.Kp;
        case 'PD'
            C = c.Kp + c.Kd * s;
        otherwise
            error('Unknown controller type: %s', c.type);
    end

    T_cl = feedback(C*G, 1);   % 폐루프 전달함수
    y = lsim(T_cl, r, t);
end
