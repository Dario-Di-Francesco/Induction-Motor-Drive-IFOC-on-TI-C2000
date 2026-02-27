%% Sampling Time
Ts = 1e-4;  % 10 kHz
Tstop = 60;  
%% Inputs (edit these with your project/nameplate data)
P_m_n     = 33e3;     % [W] rated mechanical shaft power (P2)
f_n       = 50;       % [Hz] rated electrical frequency
V_ll_n    = 400;      % [V] rated line-line RMS voltage (AC side)
p         = 1;        % [-] pole pairs (2 poles -> p = 1)
wr_n_rpm = 2950;     % [rpm] rated rotor mechanical speed
eta_n     = 0.938;    % [-] assumed rated efficiency (realistic IE3 ~33–37 kW)
V_dc       = 600;      % [V] rated Voltage DC-Link
Im_line_n = 25;        % [A] assigned magnetizing current (LINE current at nominal voltage)
alpha_Im_over_Is = 0.40;  % [-] Im_phase / Is_phase at rated point (assumption)
connection = "wye";     % "delta" or "wye" (choose consistent with your motor model)
J = 0.05;     % [kg*m^2] project data
B = 0;        % [N*m*s] viscous friction neglected
Is_adc=100;

%% Phase quantities (need these for Rs, Lm, Z_tot)
switch connection
    case "delta"
        V_ph  = V_ll_n;                % [V] delta: V_phase = V_ll
        Im_ph = Im_line_n/sqrt(3);     % [A] delta: I_phase = I_line/sqrt(3)
    case "wye"
        V_ph  = V_ll_n/sqrt(3);        % [V] wye: V_phase = V_ll/sqrt(3)
        Im_ph = Im_line_n;             % [A] wye: I_phase = I_line
    otherwise
        error("connection must be 'delta' or 'wye'");
end


%% Synchronous speed and slip
w_rpm = 60*f_n/p;                           % [rpm] synchronous speed
s_n     = (w_rpm - p*wr_n_rpm)/w_rpm;      % [-] rated slip

%% Mechanical angular speeds (rad/s)
wr_n   = 2*pi*wr_n_rpm/60;                % [rad/s] rotor mechanical angular speed
w_n = 2*pi*f_n;                  % [rad/s] synchronous mechanical angular speed


%% Signal reference Acelleration

make_ref_profile(Tstop, Ts, w_n);


%% Rated torque from mechanical power and rotor speed
T_n = P_m_n / wr_n;                        % [Nm] rated torque
T_k = 2*T_n;  % [Nm] breakdown torque (assigned via ratio)

%% Air-gap power (P_delta)
P_delta = T_n * w_n;                    % [W] air-gap power

%% Rotor copper losses (Joule losses in the rotor)
% Neglecting mechanical losses: developed mechanical power Pm ≈ P_m_n
P_jr = s_n * P_delta;               % [W] computed using slip relation

%% Input active power and total losses (only copper losses if iron+mech are neglected)
P_in   = P_m_n / eta_n;                       % [W] estimated input active power
P_loss = P_in - P_m_n;                        % [W] total losses

%% Stator copper losses estimate
% With iron + mechanical losses neglected:
P_js = P_loss - P_jr;               % [W] stator copper losses estimate

%% R_s calculation
Is_ph = Im_ph / alpha_Im_over_Is;              % [A] assumed rated stator phase current
R_s=P_js/(3*Is_ph^2); % [ohm]  stator resistance per phase

%% L_m calculation
L_m = V_ph / (w_n * Im_ph);                % [H] magnetizing inductance (first-cut)

%% X_sigma_t calculation
X_sigma_t=sqrt((((3*p*(V_ph^2))/(2*T_k*w_n))-R_s)^(2)-R_s^2);

%% L_sigma_s and L_sigma_r calculation
L_sigma_t=X_sigma_t/w_n;
L_sigma_s=L_sigma_t/2; % Stator leakage inductance
L_sigma_r=L_sigma_t/2; % Rotor leakage inductanc

%% Phi calculation

switch connection
    case "delta"
        I_line_s = sqrt(3)*Is_ph;    % [A]
    case "wye"
        I_line_s = Is_ph;            % [A]
end

cosphi = P_in/(sqrt(3)*V_ll_n*I_line_s);
cosphi = max(min(cosphi, 1), 0);       % clamp to [0,1]
phi    = acos(cosphi);                 % [rad] power-factor angle

%% R_r calculation

% 1) Total per-phase impedance seen at the terminals
Ztot_abs = V_ph/Is_ph;                 % [ohm]
Z_tot    = Ztot_abs*(cos(phi) + 1j*sin(phi)); % [ohm] (inductive load => +j)

% 2) Build Z1 and Zm from the parameters we already estimated
X_sigma_s = w_n*L_sigma_s;         % [ohm]
Z_1 = R_s + 1j*X_sigma_s;              % [ohm] stator series impedance
Z_m = 1j*w_n*L_m;                  % [ohm] magnetizing reactance (iron loss neglected)

% 3) Isolate the "parallel" part Z2 = (Zm || Zr)
Z_2 = Z_tot - Z_1;

% 4) Work in admittances: Y2 = 1/Z2 = Ym + Yr  -> Yr = Y2 - Ym
Y_2 = 1./Z_2;
Y_m = 1./Z_m;
Y_r = Y_2 - Y_m;

% 5) Rotor branch impedance (referred to stator): Zr = 1/Yr = Rr'/s + jX_sigma_r'
Z_r = 1./Y_r;

R_r_over_s = real(Z_r);               % [ohm] = Rr'/s
X_sigma_rp = imag(Z_r);               % [ohm] rotor leakage reactance (referred)

R_r  = s_n * R_r_over_s;         % [ohm] rotor resistance referred to stator

%% P_IN VA

P_IN_VA = sqrt(3) * V_ll_n * I_line_s;    % [VA] Apparent power rating (VA) for the block


%% Vboost
V_phase_peak_n=V_ll_n*sqrt(2)/sqrt(3);
V_boost=23;
%% magnetic flux

PHI_n=V_phase_peak_n/w_n;
i_m_n=PHI_n/L_m;

%% Controller Parameters

sigma_r=L_sigma_r/L_m;
sigma_s=L_sigma_s/L_m;

Tau_r=(L_m+L_sigma_r)/R_r;

% kp=0.5*L_m/Ts;
% ki=0.5*kp^2/L_m;
% Vlim=R_s*In_S+L_m*In_S/(5*Ts);

L_s = L_m + L_sigma_s;
L_r = L_m + L_sigma_r;

L_sigma = L_s - (L_m^2)/L_r;   % "transient/leakage" inductance used in dq current loop

f_bw_i = 400;                 % [Hz] current-loop bandwidth (recommended 600..1000)
alpha  = 2*pi*f_bw_i;         % [rad/s]

kp = alpha * L_sigma;         % [V/A]
ki = alpha * R_s;             % [V/(A*s)]
% lim = V_dc/2;                  % [V] saturation of PI output
lim = V_dc/sqrt(3); 

%% Decoupling Terms Parameters
K_s= L_m*((1+sigma_s)-(1/(1+sigma_r)));
K_r= L_m/(1+sigma_r);

%% filter flux
alpha_filter=1-exp(-2*pi*300*Ts);

use_peak_currents = true;
%% Parameter PI flux Controller

% --- Target specs
PO = 0.04;               % 4% overshoot (dentro 4-5%)
tr = 4 * Tau_r;        % rise time richiesto

% --- Damping ratio from overshoot
zeta = -log(PO)/sqrt(pi^2 + (log(PO))^2);

% --- Natural frequency from rise time (0-100% rise time formula)
theta = atan( sqrt(1 - zeta^2)/zeta );
omega_n = (pi - theta) / ( tr * sqrt(1 - zeta^2) );

% --- PI gains for plant 1/(Tau_r s + 1)
Kp_flux = 2*zeta*omega_n*Tau_r - 1;
Ki_flux = (omega_n^2)*Tau_r;

% ---------- Torque constant Kt for IM under rotor-flux orientation
% Te ≈ Kt * i_sq   with  Kt = (3/2)*p*(L_m^2/L_r)*i_mr_ref
% Reference magnetizing current from your nameplate assumption Im_ph (RMS phase)
imr_ref_rms  = Im_ph;                 % [A RMS phase]
imr_ref_peak = sqrt(2) * imr_ref_rms;  % [A peak phase]

if use_peak_currents
    imr_ref = imr_ref_peak;
else
    imr_ref = imr_ref_rms;
end

Kt = (3/2) * p * (L_m^2 / L_r) * imr_ref;  % [N*m / A]

% =============================
% SPEED LOOP TARGETS
% =============================
OS_w = 0.04;          % 4.5% (nel range 4-5%)
zeta_w = -log(OS_w)/sqrt(pi^2 + (log(OS_w))^2);

T_m = (J*wr_n)/(T_n);      % [s] acceleration time constant at rated torque
tr_w = 4*T_m;        % [s] richiesto

% omega_n from 0-100% rise time formula
theta = atan( sqrt(1 - zeta_w^2)/zeta_w );
omega_n_w = (pi - theta) / ( tr_w * sqrt(1 - zeta_w^2) );

% PI gains (general form with viscous friction B)
Kp_w = (2*zeta_w*omega_n_w*J)/Kt;
Ki_w = (omega_n_w^2*J)/Kt;


%% ============================================================
%  CURRENT LIMITS for SPEED/FLUX CONTROLLERS (I_s_max_in)
%  This value must be in the SAME UNITS of your i_sd / i_sq signals.
% ============================================================

% --- USER KNOBS
k_over_I = 1.5;                   % [-] overload factor (1.0 continuous, 1.5..2 short-time)
dq_scaling = "amplitude_invariant"; % "amplitude_invariant" or "power_invariant"

% --- Base RMS currents from the script
% Is_ph is the rated STATOR PHASE current you computed:
%   Is_ph = Im_ph / alpha_Im_over_Is;   % [A RMS phase]
Is_phase_rms = Is_ph;   % [A RMS] phase current (winding current)

% Many drive measurements/control use the currents at inverter terminals (LINE currents).
% If your model measures i_a,i_b,i_c at the machine terminals (typical), use line current.
switch connection
    case "delta"
        Is_line_rms = sqrt(3) * Is_phase_rms;  % delta: I_line = sqrt(3)*I_phase
    case "wye"
        Is_line_rms = Is_phase_rms;            % wye: I_line = I_phase
    otherwise
        error("connection must be 'delta' or 'wye'");
end

% --- Choose what current your controller is limiting:
% Set this to Is_line_rms if your dq currents come from measured terminal currents i_a,i_b,i_c.
% Set this to Is_phase_rms only if your dq currents are winding phase currents.
I_rms_for_control = Is_line_rms;

% --- Apply overload
I_rms_max_for_control = k_over_I * I_rms_for_control;

% --- Convert RMS ↔ peak depending on your dq convention
if use_peak_currents
    I_base_max = sqrt(2) * I_rms_max_for_control;  % [A peak]
else
    I_base_max = I_rms_max_for_control;            % [A RMS]
end

% --- Map phase/line peak to dq magnitude depending on Clarke/Park scaling
% For balanced sinusoidal currents:
% - amplitude-invariant Clarke: |i_dq|_max ≈ I_peak
% - power-invariant Clarke (sqrt(2/3) scaling): |i_dq|_max ≈ sqrt(3/2)*I_peak
switch dq_scaling
    case "amplitude_invariant"
        k_dq = 1.0;
    case "power_invariant"
        k_dq = sqrt(3/2);
    otherwise
        error("dq_scaling must be 'amplitude_invariant' or 'power_invariant'");
end

% --- Final limit in the SAME UNITS as i_sd / i_sq
I_s_max = k_dq * I_base_max;          % <-- this is the value you want
gain_wr_i_m_r= (2/3)*(1+sigma_r)*(T_n*wr_n)*(1/(p*L_m));
max_phi=V_dc/(w_n*L_m);
min_phi=0;
imr_min = 0.30*imr_ref_peak; 