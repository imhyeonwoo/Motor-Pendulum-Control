%% test_new_params.m
%  3.2.2절: 기존 K,tau 와 재추정 K,tau 비교 (PI, PD 각 1개)
%
%  - 사용하는 플랜트:
%      G(s) = K / ( s (tau s + 1) )

clear; close all; clc;

%% 1. 데이터 폴더 찾기
data_dir = find_pid_folder();
fprintf('Using data directory: %s\n', data_dir);

% 사용할 파일 (필요하면 여기만 바꿔서 다른 실험으로 테스트 가능)
pi_file = 'p1i0.5.mat';
pd_file = 'p1d0.1.mat';

%% 2. 플랜트 파라미터 설정
% (1) 기존 이론 플랜트 파라미터 (예비보고서 값)
K_nom   = 21.9;    % <- 필요하면 네 이론값으로 수정
tau_nom = 0.15;

% (2) 재추정 플랜트 파라미터 (3.2.1에서 구한 값)
[tau_new, K_new] = estimate_model_gain();   % 이미 작성한 함수

fprintf('\nNominal model   : K = %.4f, tau = %.4f\n', K_nom, tau_nom);
fprintf('Re-estimated model: K = %.4f, tau = %.4f\n\n', K_new, tau_new);

% 전달함수 생성
s = tf('s');
G_nom = K_nom  / (s * (tau_nom * s + 1));
G_new = K_new  / (s * (tau_new * s + 1));

%% 3. PI 제어 케이스
fprintf('=== PI case: %s ===\n', pi_file);
simulate_and_plot_case(fullfile(data_dir, pi_file), G_nom, G_new, 'PI Control');

%% 4. PD 제어 케이스
fprintf('\n=== PD case: %s ===\n', pd_file);
simulate_and_plot_case(fullfile(data_dir, pd_file), G_nom, G_new, 'PD Control');


%% ====== 아래는 서브함수들 ======

function data_dir = find_pid_folder()
    % 현재 위치 기준으로 4*data/PID 폴더 탐색
    search_root = pwd;
    for i = 1:4
        candidates = dir(fullfile(search_root, '4*data'));
        for c = candidates'
            if ~c.isdir, continue; end
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
    error('PID data 폴더를 찾을 수 없습니다. test_new_params.m 안에서 data_dir 을 수동으로 지정해 주세요.');
end


function simulate_and_plot_case(mat_path, G_nom, G_new, case_title)
    % mat 파일 로드
    raw = load(mat_path);

    time = raw.time(:);
    ref  = raw.signal_generated(:);
    y_exp = raw.encoder(:);

    Kp = mean(raw.Kp_simout(:));
    Ki = mean(raw.Ki_simout(:));
    Kd = mean(raw.Kd_simout(:));

    fprintf('  Controller gains: Kp = %.4f, Ki = %.4f, Kd = %.4f\n', Kp, Ki, Kd);

    % 제어기 전달함수 C(s) = Kp + Ki/s + Kd*s
    s = tf('s');
    C = Kp + Ki / s + Kd * s;

    % 폐루프 전달함수 (기존 vs 재추정 플랜트)
    T_nom = feedback(C * G_nom, 1);
    T_new = feedback(C * G_new, 1);

    % 시뮬레이션
    y_nom = lsim(T_nom, ref, time);
    y_new = lsim(T_new, ref, time);

    % RMSE 계산 (실험 대비)
    rmse_nom = sqrt(mean((y_nom - y_exp).^2));
    rmse_new = sqrt(mean((y_new - y_exp).^2));

    fprintf('  RMSE (nominal vs exp)     = %.4f deg\n', rmse_nom);
    fprintf('  RMSE (re-estimated vs exp)= %.4f deg\n', rmse_new);

    % -------- 그림 --------
    figure('Color', 'w', 'Position', [100 100 1000 400]);
    plot(time, ref, 'k--', 'LineWidth', 1.0); hold on;
    plot(time, y_exp, 'b',  'LineWidth', 1.2);
    plot(time, y_nom, 'r',  'LineWidth', 1.2);
    plot(time, y_new, 'm',  'LineWidth', 1.2);
    grid on;
    xlabel('Time [s]');
    ylabel('Angle [deg]');
    legend('Reference (input)', ...
           'Experiment (encoder)', ...
           'Simulation (nominal K,\tau)', ...
           'Simulation (re-estimated K,\tau)', ...
           'Location', 'best');
    title(sprintf('%s Response Comparison', case_title));
end
