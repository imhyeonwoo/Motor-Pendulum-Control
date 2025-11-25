## Motor-Pendulum-Control

Source code for learning DC motor position control and rotary inverted pendulum control using MATLAB/Simulink.

---
## 0. Project Structure

```text
Motor-Pendulum-Control/
├── docs/                         # Figures and reference materials
│   ├── DCmotor_pid_simulink.png
│   ├── DCmotor_PID_withoutFunction.png
│   ├── RotaryPendulum_simulink.png
│   └── RotaryPendulum_StateSpaceController.png
├── matlab/
│   ├── models/                   # Simulink models
│   │   ├── DCmotor_PID.slx
│   │   ├── InvertedPendulum_simonly.slx
│   │   └── MotorPD_simonly.slx
│   ├── scripts/                  # MATLAB parameter / setup scripts
│   │   ├── init_dc_motor.m
│   │   └── init_rotary_pendulum.m
│   └── results/                  # Simulation result plots
│       ├── DC_PID_Final.png
│       ├── DC_P_Control/
│       │   ├── Kp_0point1.png
│       │   ├── Kp_1point0.png
│       │   └── Kp_negative.png
│       ├── DC_PI_Control/
│       │   ├── BigKi.png
│       │   ├── SmallKi.png
│       │   └── unstableKi.png
│       ├── DC_PD_Control/
│       │   ├── case1.png
│       │   ├── case2.png
│       │   ├── case3.png
│       │   └── case4.png
│       └── RotaryPendulum/
│           ├── alpha_result.png
│           └── theta_result.png
├── .gitignore
└── README.md
```

---

## 1. DC Motor Position Control (P/PI/PD/PID)

- Model files: `matlab/models/DCmotor_PID.slx`, `matlab/models/MotorPD_simonly.slx`
- Parameter script: `matlab/scripts/init_dc_motor.m`

### System Model

Using the parameters $K$ and $\tau$ defined in `init_dc_motor.m`,  
the first-order system from input voltage $u(t)$ to motor angular velocity $\omega(t)$ is

$$
\tau \frac{d\omega(t)}{dt} + \omega(t) = K\,u(t)
$$

$$
\frac{\Omega(s)}{U(s)} = \frac{K}{\tau s + 1}
$$

To obtain motor position $\theta(t)$ as the output, add an integrator, giving

$$
\frac{\Theta(s)}{U(s)} = \frac{K}{s(\tau s + 1)}
$$

### Simulink Model

![DC motor PID Simulink model](docs/DCmotor_pid_simulink.png)

The controller and plant are implemented in a single Simulink model using standard blocks.

![DC motor PID (no function) model](docs/DCmotor_PID_withoutFunction.png)

### Simulation Results (PID Control)

P/PI/PD/PID control results are stored in the `matlab/results` folder.  
The figure below shows an example of the final response with PID control:

![DC motor PID final response](matlab/results/DC_PID_Final.png)

Main cases:
- `matlab/results/DC_P_Control`: responses for different proportional gains $K_p$
- `matlab/results/DC_PI_Control`: stable/unstable cases for different integral gains $K_i$
- `matlab/results/DC_PD_Control`: several PD parameter cases

---

## 2. Rotary Inverted Pendulum Control

- Model file: `matlab/models/InvertedPendulum_simonly.slx`
- Parameter script: `matlab/scripts/init_rotary_pendulum.m`

### System Model

This project uses a linearized state-space model defined in `init_rotary_pendulum.m`.  
Let the state be $x = [\theta,\ \dot\theta,\ \alpha,\ \dot\alpha]^\mathsf{T}$,  
the input $u = V_m$, output $y = [\theta,\ \alpha]^\mathsf{T}$. Then

$$
\dot{x} = A x + B u,\qquad y = C x + D u
$$

where

$$
J_\mathrm{eq} = m_p L_r^2 + J_r,\qquad
J_{p,\mathrm{eq}} = J_p + \tfrac{1}{4} m_p L_p^2,\qquad
c = \tfrac{1}{2} m_p L_p L_r,
$$

$$
a = \frac{k_m^2}{R_m} + D_r,\qquad
b = \frac{k_m}{R_m},\qquad
d = \tfrac{1}{2} m_p L_p g,
$$

$$
\Delta = J_\mathrm{eq} J_{p,\mathrm{eq}} - c^2
$$

The state-space matrices are

$$
A =
\begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & -\dfrac{J_{p,\mathrm{eq}} a}{\Delta} & \dfrac{d c}{\Delta} & \dfrac{D_p c}{\Delta} \\
0 & 0 & 0 & 1 \\
0 & \dfrac{a c}{\Delta} & -\dfrac{J_\mathrm{eq} d}{\Delta} & -\dfrac{D_p J_\mathrm{eq}}{\Delta}
\end{bmatrix},
$$

$$
B =
\begin{bmatrix}
0 \\
\dfrac{J_{p,\mathrm{eq}} b}{\Delta} \\
0 \\
-\dfrac{b c}{\Delta}
\end{bmatrix},
\quad
C =
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 0 & 1 & 0
\end{bmatrix},
\quad
D =
\begin{bmatrix}
0 \\
0
\end{bmatrix}.
$$

### Simulink Model and Controller Structure

![Rotary pendulum Simulink model](docs/RotaryPendulum_simulink.png)

![Rotary pendulum state-space controller](docs/RotaryPendulum_StateSpaceController.png)

### Simulation Results

The folder `matlab/results/RotaryPendulum` contains the response plots for the angles.

![Rotary pendulum theta response](matlab/results/RotaryPendulum/theta_result.png)

---

## How to Use (Quick Summary)

1. Start MATLAB and Simulink.
2. Set this repository as the current MATLAB working folder.
3. Initialize parameters:  
   - DC motor : run `init_dc_motor.m`  
   - Rotary pendulum : run `init_rotary_pendulum.m`
4. Open the desired `.slx` model and run the simulation.
5. Check the simulation results under the corresponding subfolders in `matlab/results`.

---