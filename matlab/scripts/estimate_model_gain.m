%% estimate_model_gain_multi.m
% P, PI, PD 세 실험 데이터를 동시에 사용해서 모터 파라미터 K, tau 역추정

clear; close all; clc;

%% 1. 기본 / 초기 파라미터
K_nom   = 21.9;
tau_nom = 0.15;

p0 = [K_nom, tau_nom];   % 초기값 [K, tau]

%% 2. 데이터 폴더 및 실험 케이스 정의
base_dir = 'C:\Users\User\Desktop\Git\Motor-Pendulum-Control\4조 data\PID';

cases = struct([]);

% (1) PI 제어기: p1i0.5.mat (Kp=1, Ki=0.5)
cases(1).file = 'p1i0.5.mat';
cases(1).type = 'PI';
cases(1).name = 'PI (Kp=1, Ki=0.5)';
cases(1).Kp   = 1;
cases(1).Ki   = 0.5;
cases(1).Kd   = 0;

% (2) P 제어기: p2.mat (Kp=2)
cases(2).file = 'p2.mat';
cases(2).type = 'P';
cases(2).name = 'P (Kp=2)';
cases(2).Kp   = 2;
cases(2).Ki   = 0;
cases(2).Kd   = 0;

% (3) PD 제어기: p1d0.05.mat (Kp=1, Kd=0.05)
cases(3).file = 'p1d0.05.mat';
cases(3).type = 'PD';
cases(3).name = 'PD (Kp=1, Kd=0.05)';
cases(3).Kp   = 1;
cases(3).Ki   = 0;
cases(3).Kd   = 0.05;

%% 3. 각 케이스별로 데이터 로드 + 사용할 구간(4~10초) 잘라서 저장
t_start = 4.0;
t_end   = 10.0;

for i = 1:numel(cases)
    c = cases(i);
    data_path = fullfile(base_dir, c.file);
    S = load(data_path);

    time    = S.time(:);
    encoder = S.encoder(:);

    if isfield(S, 'signal_generated')
        signal = S.signal_generated(:);
    elseif isfield(S, 'signal')
        signal = S.signal(:);
    else
        error('signal / signal_generated가 없습니다: %s', c.file);
    end

    idx = (time >= t_start) & (time <= t_end);

    cases(i).t_seg = time(idx);
    cases(i).r_seg = signal(idx);
    cases(i).y_seg = encoder(idx);
end

%% 4. 비용 함수 정의 (P+PI+PD 전체 RMSE 합)
cost_fun = @(p) multi_case_rmse(p, cases);

options = optimset('Display','iter', 'TolX',1e-4, 'TolFun',1e-4);
[p_hat, J_hat] = fminsearch(cost_fun, p0, options);

K_hat   = p_hat(1);
tau_hat = p_hat(2);

fprintf('\n===== Multi-case identification result =====\n');
fprintf('  K_hat   = %.3f  (initial %.3f)\n', K_hat, K_nom);
fprintf('  tau_hat = %.4f  (initial %.4f)\n', tau_hat, tau_nom);
fprintf('  Total cost (sum RMSE) = %.3f deg\n', J_hat);

%% 5. 각 케이스에 대해 새로운 파라미터로 시뮬레이션 + RMSE 비교
for i = 1:numel(cases)
    c = cases(i);

    t = c.t_seg;
    r = c.r_seg;
    y_exp = c.y_seg;

    y_nom = simulate_case_response([K_nom, tau_nom], t, r, c);
    y_new = simulate_case_response([K_hat, tau_hat], t, r, c);

    % steady-state 평균 제거 후 RMSE
    y_exp_d = y_exp   - mean(y_exp(end-200:end));
    y_nom_d = y_nom   - mean(y_nom(end-200:end));
    y_new_d = y_new   - mean(y_new(end-200:end));

    rmse_nom = sqrt(mean((y_nom_d - y_exp_d).^2));
    rmse_new = sqrt(mean((y_new_d - y_exp_d).^2));

    fprintf('\n--- %s (4~10 s 구간) ---\n', c.name);
    fprintf('  RMSE_nom (K=%.1f, tau=%.3f) = %.3f deg\n', ...
            K_nom, tau_nom, rmse_nom);
    fprintf('  RMSE_new (K=%.3f, tau=%.3f) = %.3f deg\n', ...
            K_hat, tau_hat, rmse_new);

    % 그림 한 번 그려보기 (원하면 보고서에도 사용 가능)
    figure('Name', ['Multi-ID ' c.name], 'Position', [100 100 900 500]);
    plot(t, r,      'k--', 'LineWidth', 1.0); hold on;
    plot(t, y_exp,  'b',   'LineWidth', 1.2);
    plot(t, y_nom,  'r--', 'LineWidth', 1.2);
    plot(t, y_new,  'm',   'LineWidth', 1.2);
    grid on;
    xlabel('Time [s]');
    ylabel('Angle [deg]');
    title(sprintf('%s: nominal vs new motor parameters', c.name));
    legend('Reference (Input)', ...
           'Experiment (Encoder)', ...
           sprintf('Simulation (K=%.1f, \\tau=%.3f)', K_nom, tau_nom), ...
           sprintf('Simulation (K=%.3f, \\tau=%.3f)', K_hat, tau_hat), ...
           'Location','Best');
end

%% ===== 로컬 함수들 =====

function J = multi_case_rmse(p, cases)
    % 여러 실험 케이스(P, PI, PD)를 동시에 사용하는 비용 함수
    J = 0;
    for k = 1:numel(cases)
        c = cases(k);
        t = c.t_seg;
        r = c.r_seg;
        y_exp = c.y_seg;

        y_model = simulate_case_response(p, t, r, c);

        % steady-state 성분 제거해서 과도응답 형상 위주로 비교
        tailN = min(200, floor(length(y_exp)/4));
        y_exp_d   = y_exp   - mean(y_exp(end-tailN+1:end));
        y_model_d = y_model - mean(y_model(end-tailN+1:end));

        rmse_k = sqrt(mean((y_model_d - y_exp_d).^2));
        J = J + rmse_k;   % 케이스별 RMSE를 단순 합 (원하면 가중치 곱해도 됨)
    end
end

function y = simulate_case_response(p, t, r, c)
    % 주어진 모터 파라미터 p = [K, tau] 와
    % 케이스 정보(c.type, c.Kp, c.Ki, c.Kd)를 사용해 닫힌루프 응답 계산
    K   = p(1);
    tau = p(2);

    s = tf('s');
    G = K / (s*(tau*s + 1));   % 모터 모델

    switch c.type
        case 'P'
            C = c.Kp;
        case 'PI'
            C = c.Kp + c.Ki/s;
        case 'PD'
            C = c.Kp + c.Kd*s;
        otherwise
            error('Unknown controller type: %s', c.type);
    end

    T_cl = feedback(C*G, 1);
    y = lsim(T_cl, r, t);
end
