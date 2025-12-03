function [tau, K] = estimate_model_gain()
% estimate_model_gain
%   그래프에서 읽은 두 개의 음의 피크 값을 사용하여
%   DC 모터 플랜트 G(s) = K / ( s (tau s + 1) ) 의
%   tau 와 K 를 역추정하는 함수.
%
% 사용한 데이터 (그래프에서 직접 읽은 값):
%   1번째 step : t_step1 = 0 s      → t_peak1 = 0.172 s,  y_peak1 = -66.2695 deg
%   2번째 step : t_step2 = 10 s     → t_peak2 = 10.184 s, y_peak2 = -88.2422 deg
%   정상상태 값은 두 경우 모두 y_ss = -45 deg 로 가정.
%   제어기 이득 : Kp = 2 (P control)

    % ----- 1. 피크 시각/값 -----
    t_step1  = 0.0;
    t_peak1  = 0.172;
    y_peak1  = -66.2695;

    t_step2  = 10.0;
    t_peak2  = 10.184;
    y_peak2  = -88.2422;

    y_ss     = -45.0;      % steady-state angle [deg]
    Kp       = 2;          % P 제어 이득

    % ----- 2. Tp (peak time) 평균 -----
    Tp1 = t_peak1 - t_step1;      % ≈ 0.172 s
    Tp2 = t_peak2 - t_step2;      % ≈ 0.184 s
    Tp  = mean([Tp1, Tp2]);       % 평균 Tp

    % ----- 3. 각 step에 대한 overshoot 비율 -----
    OS1 = abs(y_peak1 - y_ss) / abs(y_ss);   % 첫 번째 step
    OS2 = abs(y_peak2 - y_ss) / abs(y_ss);   % 두 번째 step

    % 두 개 평균을 사용 (퍼센트로 변환)
    OS  = mean([OS1, OS2]) * 100;           % [%]

    % ----- 4. 감쇠비, 주파수 계산 -----
    %  zeta = -ln(OS/100) / sqrt(pi^2 + ln^2(OS/100))
    zeta = -log(OS/100) / sqrt(pi^2 + log(OS/100)^2);

    %  omega_d = pi / Tp
    omega_d = pi / Tp;

    %  omega_n = omega_d / sqrt(1 - zeta^2)
    omega_n = omega_d / sqrt(1 - zeta^2);

    %  tau = 1 / (2 * zeta * omega_n)
    tau = 1 / (2 * zeta * omega_n);

    %  K = omega_n^2 * tau / Kp
    K = (omega_n^2 * tau) / Kp;

    % ----- 5. 결과 출력 -----
    fprintf('Tp1 = %.4f s, Tp2 = %.4f s,  Tp(avg) = %.4f s\n', Tp1, Tp2, Tp);
    fprintf('OS1 = %.2f %%, OS2 = %.2f %%, OS(avg) = %.2f %%\n', ...
            OS1*100, OS2*100, OS);
    fprintf('zeta    = %.6f\n', zeta);
    fprintf('omega_d = %.4f rad/s\n', omega_d);
    fprintf('omega_n = %.4f rad/s\n', omega_n);
    fprintf('tau (estimated) = %.6f s\n', tau);
    fprintf('K   (estimated) = %.6f\n', K);
end
