function [Kp_hat, Kd_hat] = estimate_PD_gains()
% estimate_PD_gains
%   PD 제어기에서 unknown control gain (Kp, Kd)을
%   step 응답 데이터로부터 역추정하는 함수.

    % --------------------------
    % 1. 플랜트 파라미터 설정
    % --------------------------
    [tau_est, K_est] = estimate_model_gain();   % 3.2.1에서 만든 함수
    K   = K_est;
    tau = tau_est;

    fprintf('Using plant: K = %.4f, tau = %.4f\n', K, tau);

    % --------------------------
    % 2. 데이터 로드
    %    ( Motor_PDcontrol_unknown_test.mat 은
    %      "4조 data" 폴더 바로 아래에 있음 )
    % --------------------------
    data_root = find_4jo_data_root();  % 4조 data 폴더
    data_file = 'Motor_PDcontrol_unknown_test.mat';

    S = load(fullfile(data_root, data_file));

    t = S.time(:);
    r = S.signal(:);      % 기준 입력
    y = S.encoder(:);     % 출력(엔코더)

    % --------------------------
    % 3. step 응답 지표 계산 (Tp, %OS)
    % --------------------------
    [Tp, OS, peak_val, y0, yf] = get_step_metrics_PD(t, r, y);

    fprintf('Step metrics from %s:\n', data_file);
    fprintf('  y0 = %.3f deg, yf = %.3f deg\n', y0, yf);
    fprintf('  peak = %.3f deg\n', peak_val);
    fprintf('  Tp   = %.4f s\n', Tp);
    fprintf('  OS   = %.2f %s\n', OS, '%%');

    % --------------------------
    % 4. ζ, ωd, ωn 계산
    % --------------------------
    zeta    = -log(OS/100) / sqrt(pi^2 + log(OS/100)^2);
    omega_d = pi / Tp;
    omega_n = omega_d / sqrt(1 - zeta^2);

    fprintf('  zeta    = %.6f\n', zeta);
    fprintf('  omega_d = %.4f rad/s\n', omega_d);
    fprintf('  omega_n = %.4f rad/s\n', omega_n);

    % --------------------------
    % 5. PD 제어기 이득 Kp, Kd 역추정
    %   특성식: tau s^2 + (1 + K Kd) s + K Kp = 0
    %   → 2ζω_n = (1 + K Kd)/tau
    %     ω_n^2 = (K Kp)/tau
    %   → Kp = (ω_n^2 * tau)/K
    %     Kd = (2 ζ ω_n tau - 1)/K
    % --------------------------
    Kp_hat = (omega_n^2 * tau) / K;
    Kd_hat = (2 * zeta * omega_n * tau - 1) / K;

    fprintf('Estimated PD gains:\n');
    fprintf('  Kp = %.6f\n', Kp_hat);
    fprintf('  Kd = %.6f\n', Kd_hat);

    % --------------------------
    % 6. 추정된 Kp, Kd로 응답 시뮬레이션 및 그림 생성
    % --------------------------
    s = tf('s');
    G_hat = K / (s * (tau * s + 1));      % 재추정 플랜트
    C_hat = Kp_hat + Kd_hat * s;          % 추정된 PD 제어기
    T_hat = feedback(C_hat * G_hat, 1);   % 폐루프 전달함수

    y_sim = lsim(T_hat, r, t);            % 모델 출력
    rmse  = sqrt(mean((y_sim - y).^2));   % 실험 vs 모델 RMSE

    fprintf('RMSE (estimated PD vs experiment) = %.4f deg\n', rmse);

    % --------- 그림(보고서용) ---------
    figure('Color','w','Position',[100 100 900 350]);
    plot(t, r, 'k--', 'LineWidth', 1.0); hold on;
    plot(t, y, 'b',   'LineWidth', 1.2);
    plot(t, y_sim, 'm', 'LineWidth', 1.4);
    grid on;
    xlabel('Time [s]');
    ylabel('Angle [deg]');
    legend('Reference (input)', ...
           'Experiment (encoder)', ...
           'Simulation (estimated K_p, K_d)', ...
           'Location', 'best');
    title('PD Unknown Control Gain Response Comparison');

    % x축/ y축 범위는 필요하면 직접 조정
    % xlim([0, max(t)]);
end


%% ===== 보조 함수들 =====

function data_root = find_4jo_data_root()
% 현재 경로 기준으로 "4*data" 폴더(예: "4조 data")를 찾아서 반환

    search_root = pwd;
    for i = 1:4
        candidates = dir(fullfile(search_root, '4*data'));
        for c = candidates'
            if ~c.isdir, continue; end
            data_root = fullfile(search_root, c.name);
            return;
        end
        parent = fileparts(search_root);
        if isempty(parent) || strcmp(parent, search_root)
            break;
        end
        search_root = parent;
    end
    error('"4조 data" 폴더를 찾지 못했습니다. find_4jo_data_root()를 직접 수정하세요.');
end


function [Tp, OS, y_peak, y0, yf] = get_step_metrics_PD(t, r, y)
% PD unknown gain 실험 데이터에서
% 첫 번째 step 응답의 Tp, %OS 계산

    N = numel(t);
    if N < 10 || any(diff(t) <= 0)
        error('시간 벡터가 비정상입니다.');
    end

    % 1) 기준 입력 r 로 step 시작 시점 찾기
    dr = diff(r);
    [max_step, idx_step] = max(abs(dr));
    if isempty(max_step) || max_step < 1e-6
        idx_step = 1;
    else
        idx_step = idx_step + 1;   % diff 보정
    end
    t0 = t(idx_step);

    % 2) 초기 / 최종값 평균
    head_n = max(10, round(0.02 * N));
    tail_n = max(20, round(0.05 * N));
    y0 = mean(y(1:head_n));
    yf = mean(y(end-tail_n+1:end));

    % 3) step 이후 구간에서 peak 찾기
    t_seg = t(idx_step:end);
    y_seg = y(idx_step:end);

    A = yf - y0;   % step 크기
    if A >= 0
        [y_peak, idx_peak_rel] = max(y_seg);  % 상승 step
    else
        [y_peak, idx_peak_rel] = min(y_seg);  % 하강 step
    end

    t_peak = t_seg(idx_peak_rel);
    Tp = t_peak - t0;

    % 4) %OS = |y_peak - yf| / |yf - y0| * 100
    OS = abs(y_peak - yf) / max(abs(A), eps) * 100;
end

%%
