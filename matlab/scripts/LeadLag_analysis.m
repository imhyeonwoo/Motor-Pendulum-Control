%% LeadLag_analysis.m
clc;

%% 1. Lead-Lag 제어기 파라미터 (5.2에서 Simulink에서 사용)

k1  = 9.1034;
k2  = 4.0327;
K_c = 1.0151;

fprintf('Lead-Lag 초기 파라미터 (변경 가능): k1=%.4f, k2=%.4f, Kc=%.4f\n', ...
        k1, k2, K_c);

%% 2. Lead-Lag 실험 데이터 로드 (5.1 데이터 분석)

data_root = find_4jo_data_root();            % "4조 data" 폴더 찾기
data_file = 'LL_unknown.mat';                % 파일 이름은 필요 시 수정

S = load(fullfile(data_root, data_file));
fprintf('Loaded Lead-Lag data from: %s\n', fullfile(data_root, data_file));

% 시간, 입력 신호, 출력 신호 추출
t = ensure_col( get_field(S, {'time','Time','t'},       'time') );
u = ensure_col( get_field(S, ...
    {'signal_generated','signal','Signal','ref'}, ...
    'input signal') );
y = ensure_col( get_field(S, {'encoder','outLeadLag','outLL','LeadLag','leadlag'}, ...
                          'Lead-Lag output') );

%% 3. Lead-Lag 실험 데이터 플롯 (5.1용)
figure('Color','w','Position',[100 100 900 350]);
plot(t, u, 'k--', 'LineWidth', 1.2); hold on;
plot(t, y, 'r',  'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Amplitude [deg]');
legend('Signal (45deg)', 'Lead-Lag Output(Encoder)', 'Location', 'best');
title('Lead-Lag 제어기 실험');

%% 4. (5.2용) 실험 vs 시뮬레이션 비교 플롯
% Simulink에서 To Workspace 블록을 Dataset 형식으로 저장했다면
%  sim 명령 후에 workspace에 'out' 이라는 SimulationOutput 변수가 생기고,
%  그 안에 out.LeadLag 라는 timeseries가 들어있다고 가정.

if exist('out','var')
    try
        sim_ts = out.LeadLag;    % timeseries
        t_sim  = sim_ts.Time;
        y_sim  = sim_ts.Data;
    catch
        % 만약 위 형식이 아니면, 직접 변수 이름을 맞춰서 수정하면 됨.
        warning('out.LeadLag를 찾지 못했습니다. To Workspace 설정을 확인하세요.');
        return;
    end

    % 실험 시간축 t에 맞게 시뮬레이션 결과 보간
    y_sim_interp = interp1(t_sim, y_sim, t, 'linear', 'extrap');

    % RMSE 계산
    rmse_LL = sqrt(mean((y_sim_interp - y).^2));
    fprintf('RMSE (Lead-Lag sim vs experiment) = %.4f deg\n', rmse_LL);

    % 비교 그림
    figure('Color','w','Position',[100 100 900 350]);
    plot(t, u, 'k--', 'LineWidth', 1.0); hold on;      % 검정 점선 (input)
    plot(t, y, 'r',  'LineWidth', 1.2);                % 빨강 실선 (encoder)
    plot(t, y_sim_interp, 'b', 'LineWidth', 1.2);      % 파랑 실선 (Simulink)
    grid on;
    xlabel('Time [s]');
    ylabel('Amplitude [deg]');
    legend('Signal (45deg)', ...
       'Lead-Lag Output(Encoder)', ...
       'Lead-Lag Sim(Simulink)', ...
       'Location', 'best');
title('Lead-Lag 제어기 실험 vs 시뮬레이션');
else
    warning('Simulink 결과(out.LeadLag)가 workspace에 없습니다. 먼저 sim을 실행하세요.');
end


%% ===== 보조 함수들 =====

function data_root = find_4jo_data_root()
    % 현재 경로 기준으로 "4* data" 폴더(예: "4조 data")를 찾아서 반환
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
    error('"4조 data" 폴더를 찾지 못했습니다. find_4jo_data_root()를 수정하세요.');
end

function v = ensure_col(x)
    v = double(x(:));
end

function value = get_field(S, names, label)
    for i = 1:numel(names)
        if isfield(S, names{i})
            value = S.(names{i});
            return;
        end
    end
    error('"%s" 변수(필드)를 데이터에서 찾을 수 없습니다.', label);
end

