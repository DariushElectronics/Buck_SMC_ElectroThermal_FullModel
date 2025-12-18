%% ========================================================================
%  Buck DC-DC Converter with Detailed Electro-Thermal Modeling
%  Sliding Mode Control + Loss Analysis + Junction Temperature
%
%  Industrial / Research Grade MATLAB Implementation
%
%  Author: ---
%  Repository: Buck-SMC-ElectroThermal-FullModel
%
%  Description:
%  High-fidelity simulation of a buck DC-DC converter with:
%  - Digital Sliding Mode Control (SMC) with boundary layer
%  - Full loss modeling (inductor, MOSFET, diode, switching)
%  - Electro-thermal modeling (MOSFET and diode junction temperature)
%  - Detailed performance reporting and professional visualization
% ========================================================================

clear; clc; close all;

%% ========================================================================
% 1. SYSTEM ELECTRICAL PARAMETERS
% ========================================================================

Vin   = 24;        % Input voltage [V]
Vref  = 12;        % Reference voltage [V]
Rload = 10;        % Load resistance [Ohm]

L     = 150e-6;    % Inductor [H]
C     = 220e-6;    % Capacitor [F]
RL    = 0.08;      % Inductor ESR [Ohm]

Rds   = 0.04;      % MOSFET Rds(on) [Ohm]
Vd    = 0.7;       % Diode forward voltage [V]

tr    = 40e-9;     % MOSFET rise time [s]
tf    = 40e-9;     % MOSFET fall time [s]

lambda = 800;      % Sliding surface coefficient
eta    = 0.01;     % Boundary layer thickness (chattering reduction)

fsw   = 50e3;      % Switching frequency [Hz]
Ts    = 1/fsw;
Tsim  = 0.03;
dt    = Ts/40;     % Integration timestep
t     = 0:dt:Tsim;
N     = length(t);

%% ========================================================================
% 2. THERMAL PARAMETERS
% ========================================================================

T_ambient  = 25;       % Ambient temperature [°C]
Rth_jc_MOS = 0.5;      % Junction-to-case thermal resistance MOSFET [°C/W]
Cth_MOS    = 5e-3;     % Thermal capacitance MOSFET [J/°C]
Rth_jc_D   = 2;        % Junction-to-case thermal resistance Diode [°C/W]
Cth_D      = 2e-3;     % Thermal capacitance Diode [J/°C]

%% ========================================================================
% 3. STATE VARIABLE INITIALIZATION
% ========================================================================

iL       = zeros(1,N);    % Inductor current [A]
Vo       = zeros(1,N);    % Output voltage [V]
S        = zeros(1,N);    % Switching signal (0-1)
Tj_MOS   = zeros(1,N);    % MOSFET junction temperature [°C]
Tj_D     = zeros(1,N);    % Diode junction temperature [°C]
int_err  = 0;             % Integral of voltage error

% Preallocate loss arrays
P_L      = zeros(1,N);
P_MOS    = zeros(1,N);
P_D      = zeros(1,N);
P_sw     = zeros(1,N);

%% ========================================================================
% 4. MAIN SIMULATION LOOP (ELECTRO-THERMAL + SMC)
% ========================================================================

for k = 2:N
    %% ----------------- Voltage Error -----------------
    err = Vref - Vo(k-1);
    int_err = int_err + err*dt;

    %% ----------------- Sliding Mode Control with Boundary Layer -----------------
    s = err + lambda*int_err;

    if s > eta
        S(k) = 1;
    elseif s < -eta
        S(k) = 0;
    else
        % Linear interpolation inside boundary layer (reduces chattering)
        S(k) = 0.5 + s/(2*eta);
    end

    %% ----------------- Buck Converter Dynamics -----------------
    diL = (1/L)*( S(k)*Vin - Vo(k-1) - iL(k-1)*RL );
    dVo = (1/C)*( iL(k-1) - Vo(k-1)/Rload );

    iL(k) = iL(k-1) + diL*dt;
    Vo(k) = Vo(k-1) + dVo*dt;

    %% ----------------- Instantaneous Losses -----------------
    IL_abs = abs(iL(k));

    % Inductor copper loss
    P_L(k) = IL_abs^2 * RL;

    % MOSFET conduction loss
    P_MOS(k) = S(k)*IL_abs^2*Rds;

    % Diode conduction loss
    I_D = (1-S(k))*Vo(k)/Rload;
    P_D(k) = abs(I_D) * Vd;

    % Switching loss
    P_sw(k) = 0.5*Vin*IL_abs*(tr+tf)*fsw;

    %% ----------------- Electro-Thermal Model -----------------
    % MOSFET junction temperature
    dTj_MOS = (P_MOS(k) + P_sw(k) - (Tj_MOS(k-1)-T_ambient)/Rth_jc_MOS)/Cth_MOS;
    Tj_MOS(k) = Tj_MOS(k-1) + dTj_MOS*dt;

    % Diode junction temperature
    dTj_D = (P_D(k) - (Tj_D(k-1)-T_ambient)/Rth_jc_D)/Cth_D;
    Tj_D(k) = Tj_D(k-1) + dTj_D*dt;
end

%% ========================================================================
% 5. STEADY-STATE ANALYSIS
% ========================================================================

idx_ss = round(0.8*N):N;

Vout_avg    = mean(Vo(idx_ss));
Vout_ripple = max(Vo(idx_ss)) - min(Vo(idx_ss));
IL_rms      = rms(iL(idx_ss));
Iout_avg    = Vout_avg / Rload;
D_avg       = mean(S(idx_ss));

Pout        = Vout_avg * Iout_avg;
P_L_total   = mean(P_L(idx_ss));
P_MOS_total = mean(P_MOS(idx_ss)) + mean(P_sw(idx_ss));
P_D_total   = mean(P_D(idx_ss));
Ploss_total = P_L_total + P_MOS_total + P_D_total;
Efficiency  = Pout / (Pout + Ploss_total);

Tj_MOS_max  = max(Tj_MOS(idx_ss));
Tj_D_max    = max(Tj_D(idx_ss));

%% ========================================================================
% 6. PROFESSIONAL VISUALIZATION
% ========================================================================

figure('Color','w','Position',[50 50 1500 900])

subplot(3,2,1)
plot(t,Vo,'LineWidth',2); hold on
yline(Vref,'--r','LineWidth',1.5)
grid on; xlabel('Time [s]'); ylabel('V_o [V]')
title('Output Voltage Regulation')

subplot(3,2,2)
plot(t,iL,'LineWidth',2)
grid on; xlabel('Time [s]'); ylabel('i_L [A]')
title('Inductor Current')

subplot(3,2,3)
stairs(t,S,'LineWidth',1.2)
grid on; xlabel('Time [s]'); ylabel('Switch Duty')
title('Switching Signal (SMC)')

subplot(3,2,4)
plot(t,Tj_MOS,'LineWidth',2); hold on
plot(t,Tj_D,'LineWidth',2)
yline(T_ambient,'--k','Ambient','LineWidth',1)
grid on; xlabel('Time [s]'); ylabel('Temperature [°C]')
legend('MOSFET Junction','Diode Junction','Ambient')
title('Electro-Thermal Response')

subplot(3,2,[5 6])
bar([P_L_total P_MOS_total P_D_total])
set(gca,'XTickLabel',{'Inductor','MOSFET+Switch','Diode'})
ylabel('Power Loss [W]')
title('Average Loss Breakdown (Steady-State)')
grid on

%% ========================================================================
% 7. CONSOLE REPORT
% ========================================================================

fprintf('\n========== BUCK CONVERTER FULL ELECTRO-THERMAL REPORT ==========\n');
fprintf('Average Output Voltage      : %.4f V\n', Vout_avg);
fprintf('Output Voltage Ripple       : %.4f V\n', Vout_ripple);
fprintf('Inductor RMS Current        : %.4f A\n', IL_rms);
fprintf('Output Power                : %.4f W\n', Pout);
fprintf('Inductor Loss               : %.4f W\n', P_L_total);
fprintf('MOSFET + Switching Loss     : %.4f W\n', P_MOS_total);
fprintf('Diode Loss                  : %.4f W\n', P_D_total);
fprintf('Total Power Loss            : %.4f W\n', Ploss_total);
fprintf('Overall Efficiency          : %.2f %%\n', Efficiency*100);
fprintf('Max MOSFET Junction Temp    : %.2f °C\n', Tj_MOS_max);
fprintf('Max Diode Junction Temp     : %.2f °C\n', Tj_D_max);
fprintf('Ambient Temperature         : %.2f °C\n', T_ambient);
fprintf('==============================================================\n');
