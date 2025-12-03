%% 1. 실험 데이터 로드 (Load Experiment Data)
base_dir = 'C:\Users\User\Desktop\Git\Motor-Pendulum-Control/4조 data/P,PI,PD test';
%file_name = 'p2.mat';
%file_name = 'p0.366.mat';
%file_name = 'p1i0.5.mat';
%file_name = 'p1i1.mat';
%file_name = 'p1d0.1.mat';
file_name = 'p1d0.05.mat';
%file_name = 'p0.5d0.1.mat';
data_path = fullfile(base_dir, file_name);

clc;

% --- P 제어기 이득 (실험 파일에 맞게 설정) ---
Kp = 1;           % p2.mat → 2, p0.366.mat 쓰면 0.366 으로 바꿔줘
Ki = 0;         
Kd = 0.05;

if isfile(data_path)
    raw_data = load(data_path); % 데이터를 구조체로 로드
    fprintf('실험 데이터 로드 성공: %s\n', data_path);
else
    error('파일을 찾을 수 없습니다: %s\n경로를 확인해주세요.', data_path);
end

% --- 변수 자동 매핑 (Auto-mapping variables) ---
vars = fieldnames(raw_data);

%% 1-1) 기준 입력 (Reference) 찾기
if isfield(raw_data, 'signal')
    signal = raw_data.signal;
elseif isfield(raw_data, 'Signal')
    signal = raw_data.Signal;
elseif isfield(raw_data, 'signal_generator')
    signal = raw_data.signal_generator;
else
    idx = find(contains(lower(vars), 'signal'), 1);
    if ~isempty(idx)
        signal = raw_data.(vars{idx});
        fprintf("알림: 기준 입력 변수로 '%s'를 사용합니다.\n", vars{idx});
    else
        error("데이터 파일에서 기준 입력(signal) 변수를 찾을 수 없습니다. 변수명을 확인하세요.");
    end
end

%% 1-2) 엔코더 출력 (Output) 찾기
if isfield(raw_data, 'Encoder')
    Encoder = raw_data.Encoder;
elseif isfield(raw_data, 'encoder')
    Encoder = raw_data.encoder;
elseif isfield(raw_data, 'theta')
    Encoder = raw_data.theta;
else
    idx = find(contains(lower(vars), 'encoder'), 1);
    if ~isempty(idx)
        Encoder = raw_data.(vars{idx});
        fprintf("알림: 엔코더 출력 변수로 '%s'를 사용합니다.\n", vars{idx});
    else
        error("데이터 파일에서 출력(Encoder) 변수를 찾을 수 없습니다.");
    end
end

%% 1-3) 시간 (Time) 찾기
if isfield(raw_data, 'time')
    time = raw_data.time;
elseif isfield(raw_data, 'Time')
    time = raw_data.Time;
elseif isfield(raw_data, 't')
    time = raw_data.t;
else
    idx = find(contains(lower(vars), 'time'), 1);
    if ~isempty(idx)
        time = raw_data.(vars{idx});
        fprintf("알림: 시간 변수로 '%s'를 사용합니다.\n", vars{idx});
    else
        fprintf("경고: 시간 변수를 찾을 수 없습니다. 0.001초 간격으로 임의 생성합니다.\n");
        time = (0:length(signal)-1)' * 0.001; 
    end
end

time      = time(:);      % 열벡터로 정리
signal    = signal(:);
Encoder   = Encoder(:);

%% 2. 전달함수 기반 P제어 시스템 직접 시뮬레이션
% Simulink:  P 제어기 → K/(tau*s+1) → 1/s → θ(rad) → deg 변환 구조와 동일하게 구현

% --- Plant parameters (from lecture slides) ---
K   = 21.9;    % DC motor gain
tau = 0.15;    % time constant [s]

N = length(time);
sim_time = time;          % 편의상 이름 통일

% 상태: x1 = θ(rad), x2 = θ_dot(rad/s)
x1 = zeros(N,1);
x2 = zeros(N,1);

% 초기각을 실험과 맞추고 싶으면:
x1(1) = Encoder(1) * pi/180;   % deg → rad
x2(1) = 0;                     % 초기 각속도는 0 가정

% PID 내부 상태 (지금은 P만 쓰지만 구조는 잡아둠)
e_int    = 0;                  % ∫e dt
e_prev   = 0;                  % 이전 오차

for k = 1:N-1
    dt = sim_time(k+1) - sim_time(k);
    if dt <= 0
        dt = 1e-3;             % 혹시라도 0이나 음수면 보호
    end

    % 기준 입력: 실험 데이터와 동일 (deg → rad)
    r_deg = signal(k);
    r     = r_deg * pi/180;

    % 출력: 현재 θ
    y = x1(k);

    % 오차
    e = r - y;

    % PID 계산 (단순 이산형)
    e_int  = e_int + e*dt;
    e_der  = (e - e_prev)/dt;
    e_prev = e;

    u = Kp*e + Ki*e_int + Kd*e_der;   % 제어입력 (전압 등)

    % 모터 모델 θ¨ = -(1/τ) θ˙ + (K/τ) u
    x1_dot = x2(k);
    x2_dot = -(1/tau)*x2(k) + (K/tau)*u;

    % Forward Euler 적분
    x1(k+1) = x1(k) + x1_dot*dt;
    x2(k+1) = x2(k) + x2_dot*dt;
end

% 시뮬레이션 결과 (deg)
sim_y = x1 * 180/pi;
sim_output_deg = sim_y;   %#ok<NASGU>  % 혹시 나중에 쓸 수도 있으니 남겨둠

%% 3. 결과 비교 및 도시 (Plot Results)
figure('Name', 'P Control Comparison', 'Color', 'w', 'Position', [100 100 800 600]);

%% 3-1) 전체 시간 응답
subplot(2,1,1); hold on;
h1 = plot(time, signal,  'k--', 'LineWidth', 1.2, 'DisplayName', 'Reference (Input)');
h2 = plot(time, Encoder, 'b-',  'LineWidth', 1.5, 'DisplayName', 'Experiment (Encoder)');
h3 = plot(sim_time, sim_y, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Simulation (model)');
grid on;
legend([h1 h2 h3], 'Location', 'best');
xlabel('Time [s]');
ylabel('Angle [deg]');
% title(sprintf('P Control Response Comparison (Kp=%.3g)', Kp));
% title(sprintf('PI Control Response Comparison (Kp=%.3g , Ki=%.3g)', Kp, Ki));
title(sprintf('PD Control Response Comparison (Kp=%.3g , Ki=%.3g)', Kp, Kd));
xlim([0, time(end)]);

%% 3-2) 확대된 응답 (상승 구간)
subplot(2,1,2); hold on;
hz1 = plot(time, signal,  'k--', 'LineWidth', 1.2, 'DisplayName', 'Reference (Input)');
hz2 = plot(time, Encoder, 'b-',  'LineWidth', 1.5, 'DisplayName', 'Experiment (Encoder)');
hz3 = plot(sim_time, sim_y, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Simulation (model)');
grid on;
legend([hz1 hz2 hz3], 'Location', 'best');
xlabel('Time [s]');
ylabel('Angle [deg]');
title('Zoomed View (Transient Response)');
xlim([0, 5]);   % 0~5초 구간 확대

%% 4. 성능 지표 계산 (RMSE)
if length(Encoder) == length(sim_y)
    rmse = sqrt(mean((Encoder - sim_y).^2));
    fprintf('실험값과 시뮬레이션값의 RMSE: %.4f deg\n', rmse);
else
    fprintf('데이터 길이가 달라 RMSE 계산을 건너뜁니다.\n');
end

%% 5. OS 및 정상상태 오차(Steady-state Error) 계산
% 1) 기준 입력(signal)에서 "가장 큰 스텝" 위치 자동 검출
ds = diff(signal);                     % 차분
[~, idx_step] = max(abs(ds));          % 변화량이 가장 큰 지점 = 주요 스텝
t_step = time(idx_step + 1);           % 스텝 발생 시각

% 2) 스텝 이후 일정 구간(예: 3초)만 분석
T_window = 3.0;                         % 스텝 이후 분석할 시간 [s]
t_start = t_step;
t_end   = min(t_step + T_window, time(end));

idx_exp = (time >= t_start) & (time <= t_end);
ref_seg = signal(idx_exp);              % 기준 입력 구간
exp_seg = Encoder(idx_exp);             % 실험 응답 구간

% 3) 기준값 R(목표값)과 steady-state 값 추정
n_exp    = numel(ref_seg);
tail_len = max(10, round(0.2 * n_exp));  % 뒤에서 20% 구간(최소 10개 샘플)

R        = mean(ref_seg(end-tail_len+1:end));      % 기준 입력의 목표값
y_exp_ss = mean(exp_seg(end-tail_len+1:end));      % 실험 응답 steady-state

% 4) Overshoot 계산 (R 기준)
if abs(R) < 1e-6
    OS_exp = NaN;    % R이 0이면 정의 애매
else
    if R >= 0
        y_exp_peak = max(exp_seg);
    else
        y_exp_peak = min(exp_seg);
    end
    OS_exp = 100 * abs(y_exp_peak - R) / abs(R);
end

% 5) Steady-state error (e_ss = R - y_inf)
ess_exp = R - y_exp_ss;

fprintf('\n[Experiment]\n');
fprintf('  Step time           : %.4f s\n', t_step);
fprintf('  Reference (R)       : %.4f deg\n', R);
fprintf('  Steady-state value  : %.4f deg\n', y_exp_ss);
fprintf('  Overshoot (OS)      : %.2f %%\n', OS_exp);
fprintf('  Steady-state error  : %.4f deg\n', ess_exp);

% ---- Simulation도 같은 구간으로 계산 ----
idx_sim = (sim_time >= t_start) & (sim_time <= t_end);
sim_seg = sim_y(idx_sim);

n_sim        = numel(sim_seg);
tail_len_sim = max(10, round(0.2 * n_sim));
y_sim_ss     = mean(sim_seg(end-tail_len_sim+1:end));   % steady-state

if abs(R) < 1e-6
    OS_sim = NaN;
else
    if R >= 0
        y_sim_peak = max(sim_seg);
    else
        y_sim_peak = min(sim_seg);
    end
    OS_sim = 100 * abs(y_sim_peak - R) / abs(R);
end

ess_sim = R - y_sim_ss;

fprintf('\n[Simulation]\n');
fprintf('  Steady-state value  : %.4f deg\n', y_sim_ss);
fprintf('  Overshoot (OS)      : %.2f %%\n', OS_sim);
fprintf('  Steady-state error  : %.4f deg\n\n', ess_sim);
