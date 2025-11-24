%% Rotary Inverted Pendulum parameters

Rm  = 8.4;        % [Ohm] terminal resistance
km  = 0.042;      % [V/(rad/s)] back-emf constant
kt  = 0.042;      % [N*m/A] torque constant
Vm  = 1.0;        % [V] motor voltage (입력 스텝 크기)

mp  = 0.024;      % [kg] pendulum mass
Lr  = 0.085;      % [m] arm length
Lp  = 0.129;      % [m] pendulum length
Jr  = 5.72e-5;    % [kg*m^2] arm inertia
Jp  = 3.33e-5;    % [kg*m^2] pendulum inertia

Dr  = 0.0015;     % [N*m*s/rad] arm viscous damping
Dp  = 0.0005;     % [N*m*s/rad] pendulum damping
g   = 9.81;       % [m/s^2] gravity

%% Convenience parameters
J_eq  = mp*Lr^2 + Jr;
Jp_eq = Jp + 0.25*mp*Lp^2;
c     = 0.5*mp*Lp*Lr;

a  = km^2/Rm + Dr;
b  = km/Rm;
d = 0.5*mp*Lp*g;

Delta = J_eq*Jp_eq - c^2;  % determinant -> use to solve theta" and alpha"

%% State-space matrices: x = [theta; dtheta; alpha; dalpha], u = Vm

A1 = zeros(4);
B1 = zeros(4,1);

% x1_dot = dtheta
A1(1,2) = 1;

% x2_dot = d2theta
A1(2,2) = -Jp_eq*a/Delta;
A1(2,3) =  d*c/Delta;
A1(2,4) =  Dp*c/Delta;
B1(2,1) =  Jp_eq*b/Delta;

% x3_dot = dalpha
A1(3,4) = 1;

% x4_dot = d2alpha
A1(4,2) =  a*c/Delta;
A1(4,3) = -J_eq*d/Delta;
A1(4,4) = -Dp*J_eq/Delta;
B1(4,1) = -b*c/Delta;

% 출력 선택
C1 = [1 0 0 0;   % theta
      0 0 1 0];  % alpha
D1 = zeros(2,1);