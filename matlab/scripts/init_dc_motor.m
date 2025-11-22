% --- Plant parameters (from lecture slides) ---
K   = 21.9;   % DC motor gain [rad/(V*s)]
tau = 0.15;   % time constant [s]

% --- (옵션) Controller gains: 나중에 튜닝하면서 값 바꿔 쓸 것 ---
% 기본값만 대충 넣어두고, Simulink에서 블록 Gain에 직접 입력해도 됨.
%Kp_P  = 1.0;   % P controller
%Kp_PI = 1.0;   % PI controller - proportional
%Ki_PI = 0.0;   % PI controller - integral
%Kp_PD = 1.0;   % PD controller - proportional
%Kd_PD = 0.0;   % PD controller - derivative