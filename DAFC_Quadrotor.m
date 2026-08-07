%% =========================================================================
%% 3D Trajectory Control Simulation for Quadrotor (COMPLETED VERSION)
%% =========================================================================
function DAFC_Quadrotor()
%% =========================================================================
%% 1. PARAMETER INITIALIZATION
%% =========================================================================
clearvars; clc; close all;
% Physical parameters
m = 1.121;         % Quadrotor mass (kg)
g = 9.81;          % Gravity acceleration (m/s^2)

Ixx = 0.01;          % Moments of inertia (kg*m^2)
Iyy = 0.01;
Izz = 0.0148;
Ir = 2.83e-5;       % Rotor moment of inertia

k = 2.98e-5;       % Lift constant
b = 3.23e-7;       % Drag constant
l = 0.25;          % Arm length (m)

Ax = 5.56e-4;       % Drag coefficients
Ay = 5.56e-4;
Az = 6.35e-4;
dt = 0.01;

%% =========================================================================
%% Parameters for Positions Controller and Attitudes Controller
%% =========================================================================
pnp = [5;5];
gamma_p = 2.5;

pna = [3;3.8];
gamma_a = 100;

c_x = - 2.5: 1: 2.5;
w_x = 0.5;
c_x_dot = -2: 0.8: 2;
w_x_dot = 0.4;

c_y = - 2.5: 1: 2.5;
w_y = 0.5;
c_y_dot = -2: 0.8: 2;
w_y_dot = 0.4;

c_z = 0: 0.6: 3;
w_z = 0.3;
c_z_dot = -1.5: 0.6: 1.5;
w_z_dot = 0.3;

c_phi =-pi/9:pi/18:pi/9;
w_phi = pi/36;
c_phi_dot =-1.5:0.6:1.5;
w_phi_dot = 0.3;

c_theta =-pi/9:pi/18:pi/9;
w_theta = pi/36;
c_theta_dot =-1.5:0.6:1.5;
w_theta_dot = 0.3;

c_psi = -10^-3 : 5*10^-4 : 10^-3;
w_psi = 2.5*10^-4;
c_psi_dot = -0.01 : 4*10^-3 : 0.01;
w_psi_dot = 2*10^-3;

%% =========================================================================
%% 2. INITIAL CONDITIONS & SIMULATION TIMING
%% =========================================================================
% Initial state vector x0 (12 x 1)
% [x, y, z, vx, vy, vz, phi, theta, psi, phi_dot, theta_dot, psi_dot,
%  Theta_x, Theta_y, Theta_z, Theta_phi, Theta_theta, Theta_psi,
%  Theta_x_dot, Theta_y_dot, Theta_z-dot, Theta_phi_dot, Theta_theta_dot, Theta_psi_dot]

x0 = zeros(212, 1);
x0(3,1) = 2;

t0 = 0;         
Tf = 50;        

%% =========================================================================
%% 3. DIFFERENTIAL EQUATION SOLVER (ODE45)
%% =========================================================================
opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', 0.01);

% Function handle directly calls nested function 'f'
[t, X] = ode45(@f, [t0 Tf], x0, opts);

% Extract states
x = X(:,1);
y = X(:,2);
z = X(:,3);

phi = X(:,7);
theta = X(:,8);
psi = X(:,9);

phi_d_prev = X(:,13);
theta_d_prev = X(:,14);

Theta_x = X(:, 15:50);
Theta_y = X(:, 51:86);
Theta_z = X(:, 87:122); %121: 156; 157:192; 193:228

Theta_phi = X(:, 123:152);
Theta_theta = X(:, 153:182);
Theta_psi = X(:, 183:212); %319: 348; 349:378; 379:408

%% =========================================================================
%% 4. REFERENCE TRAJECTORY GENERATION FOR PLOTTING
%% =========================================================================
xd = 0.5*cos(pi*t/20);
yd = 0.5*sin(pi*t/20);
zd = 2 - 0.5*cos(pi*t/20);
psi_d = zeros(size(t));

%% =========================================================================
%% 5. PLOTTING RESULTS
%% =========================================================================
figure('Name','3D Trajectory');
plot3(x, y, z, 'b', 'LineWidth', 2); hold on;
plot3(xd, yd, zd, 'r--', 'LineWidth', 2);
grid on;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
legend('Actual', 'Desired');
title('3D Trajectory Tracking');
axis equal; view(45, 30);

figure('Name','Position tracking');
subplot(3,1,1);
plot(t, x, 'b', t, xd, 'r--', 'LineWidth', 1.5); grid on;
ylabel('x (m)'); legend('Actual','Desired');
title('Position tracking');
subplot(3,1,2);
plot(t, y, 'b', t, yd, 'r--', 'LineWidth', 1.5); grid on;
ylabel('y (m)'); legend('Actual','Desired')
subplot(3,1,3);
plot(t, z, 'b', t, zd, 'r--', 'LineWidth', 1.5); grid on;
ylabel('z (m)'); xlabel('Time (s)');  legend('Actual','Desired');

figure('Name','Attitude tracking','Color','w');
subplot(3,1,1);
plot(t, phi, 'b', t, phi_d_prev, 'r--', 'LineWidth', 1.5); grid on;
ylabel('\phi (rad)'); legend('\phi_d','\phi');
title('Attitude tracking');
subplot(3,1,2);
plot(t, theta, 'b', t, theta_d_prev, 'r--', 'LineWidth', 1.5); grid on;
ylabel('\theta (rad)'); legend('\theta_d','\theta');
subplot(3,1,3);
plot(t, psi, 'b', t, psi_d, 'r--', 'LineWidth', 1.5); grid on;
ylabel('\psi (rad)'); xlabel('Time (s)'); legend('\psi','\psi_d');
%% =========================================================================
%% CONTROL FUNCTION
%% =========================================================================
function deq = f(t, X)  

    % Reference Trajectory
    xd = 0.5*cos(pi*t/20);
    yd = 0.5*sin(pi*t/20);
    zd = 2 - 0.5*cos(pi*t/20);
    psi_d = 0;
    psi_d_dot = 0;

    xd_dot = -(0.5*pi/20)*sin(pi*t/20);
    yd_dot = (0.5*pi/20)*cos(pi*t/20);
    zd_dot = (0.5*pi/20)*sin(pi*t/20);

    x = X(1); y = X(2); z = X(3);
    x_dot = X(4); y_dot = X(5); z_dot = X(6);

    phi = X(7); theta = X(8); psi = X(9);
    phi_dot = X(10); theta_dot = X(11); psi_dot = X(12);

    phi_d_prev = X(13);
    theta_d_prev = X(14);

    Theta_x = X(15:50);
    Theta_y = X(51:86);
    Theta_z = X(87:122);

    Theta_phi = X(123:152);
    Theta_theta = X(153:182);
    Theta_psi = X(183:212);

    % External Disturbances
    dx = 0.3*sin(t);
    dy = 0.3*sin(t);
    dz = 0.1*cos(0.5*t);
    dphi = 0.2*sin(2*t);
    dtheta = 0.2*sin(2*t);
    dpsi = 0.1*sin(t);

%% =========================================================================
%% Outer Loop: Position Control
%% =========================================================================
    ex = xd - x;
    ey = yd - y;
    ez = zd - z;
    
    ex_dot = xd_dot - x_dot;
    ey_dot = yd_dot - y_dot;
    ez_dot = zd_dot - z_dot;
    
    zeta_x = zeta_p(x, x_dot, c_x, w_x, c_x_dot, w_x_dot);
    zeta_y = zeta_p(y, y_dot, c_y, w_y, c_y_dot, w_y_dot);
    zeta_z = zeta_p(z, z_dot, c_z, w_z, c_z_dot, w_z_dot);

    Theta_x_dot = gamma_p * (ex*pnp(1) + ex_dot*pnp(2)) * zeta_x;    
    Theta_y_dot = gamma_p * (ey*pnp(1) + ey_dot*pnp(2)) * zeta_y;
    Theta_z_dot = gamma_p * (ez*pnp(1) + ez_dot*pnp(2)) * zeta_z;    
    
    ux = Theta_x.' * zeta_x;   
    uy = Theta_y.' * zeta_y;
    uz = Theta_z.' * zeta_z;
%% =========================================================================
%% Inner Loop: Attitude Control
%% =========================================================================

%% Thrust and Desired Roll/Pitch

    T = m*sqrt(ux^2 + uy^2 + (uz + g)^2); 
    
    phi_d = asin((ux*sin(psi_d) - uy*cos(psi_d)) / sqrt(ux^2 + uy^2 + (uz + g)^2));

    theta_d = atan((ux*cos(psi_d) + uy*sin(psi_d)) / (uz + g));

    phi_d_dot = (phi_d  - phi_d_prev)/dt;
    theta_d_dot = (theta_d - theta_d_prev)/dt;
    
    
    e_phi = phi_d - phi;
    e_theta = theta_d - theta;
    e_psi = psi_d - psi;

    e_phi_dot = phi_d_dot - phi_dot;
    e_theta_dot = theta_d_dot - theta_dot;
    e_psi_dot = psi_d_dot - psi_dot;

    zeta_phi = zeta_a(phi, phi_dot, c_phi, w_phi, c_phi_dot, w_phi_dot);
    zeta_theta = zeta_a(theta, theta_dot, c_theta, w_theta, c_theta_dot, w_theta_dot);
    zeta_psi = zeta_a(psi, psi_dot, c_psi, w_psi, c_psi_dot, w_psi_dot);

    Theta_phi_dot = gamma_a * (e_phi*pna(1)+e_phi_dot*pna(2))*zeta_phi;
    Theta_theta_dot = gamma_a * (e_theta*pna(1) + e_theta_dot*pna(2))*zeta_theta;
    Theta_psi_dot = gamma_a * (e_psi*pna(1) + e_psi_dot*pna(2))*zeta_psi;
    
    tau_phi = Theta_phi.' * zeta_phi;
    tau_theta = Theta_theta.' * zeta_theta;
    tau_psi = Theta_psi.' * zeta_psi;

% Motor speeds
    w1 = sqrt(max(0,T/(4*k) - tau_theta/(2*k*l) - tau_psi/(4*b)));
    w2 = sqrt(max(0,T/(4*k) - tau_phi  /(2*k*l) + tau_psi/(4*b)));
    w3 = sqrt(max(0,T/(4*k) + tau_theta/(2*k*l) - tau_psi/(4*b)));
    w4 = sqrt(max(0,T/(4*k) + tau_phi  /(2*k*l) + tau_psi/(4*b)));

    w_alpha = w1 - w2 + w3 - w4;

    %% Derivative equations of attitudes

    phi_dot2 = theta_dot*psi_dot*(Iyy - Izz)/Ixx - theta_dot*w_alpha*Ir/Ixx + tau_phi/Ixx + dphi;

    theta_dot2 = phi_dot*psi_dot*(Izz - Ixx)/Iyy + phi_dot*w_alpha*Ir/Iyy + tau_theta/Iyy + dtheta;

    psi_dot2 = phi_dot*theta_dot*(Ixx - Iyy)/Izz + tau_psi/Izz + dpsi;
    %% Derivative equations of positions

    x_dot2 = (T/m)*(cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)) - (Ax/m)*x_dot + dx;

    y_dot2 = (T/m)*(cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)) - (Ay/m)*y_dot + dy;

    z_dot2 = (T/m)*cos(phi)*cos(theta) - g - (Az/m)*z_dot + dz;

    % Derivatives Vector Output
    deq = zeros(210,1);
    
    deq(1) = x_dot;
    deq(2) = y_dot;
    deq(3) = z_dot;

    deq(4) = x_dot2;
    deq(5) = y_dot2;
    deq(6) = z_dot2;

    deq(7) = phi_dot;
    deq(8) = theta_dot;
    deq(9) = psi_dot;

    deq(10) = phi_dot2;
    deq(11) = theta_dot2;
    deq(12) = psi_dot2;

    deq(13) = phi_d_dot;
    deq(14) = theta_d_dot;

    deq(15:50) = Theta_x_dot;
    deq(51:86) = Theta_y_dot;
    deq(87:122) = Theta_z_dot;
    
    deq(123:152)=Theta_phi_dot;
    deq(153:182)=Theta_theta_dot;
    deq(183:212)=Theta_psi_dot;

end
end

function zeta_p = zeta_p(x, x_dot, c_x, w_x, c_x_dot, w_x_dot)

mu_p = zeros(1, 6);

for i = 1:6
mu_p(i) = exp(-(x - c_x(i))^2 / (2 * w_x^2));
end

mu_v = zeros(1, 6);

for i = 1:6
mu_v(i) = exp(-(x_dot - c_x_dot(i))^2 / (2* w_x_dot^2));
end

zeta_p = zeros(36, 1);
k = 1;

for i = 1:6
    for j = 1:6
    zeta_p(k) = mu_p(i) * mu_v(j);
    k = k + 1;
    end
end
end

function zeta_a = zeta_a(phi, phi_dot, c_phi, w_phi, c_phi_dot, w_phi_dot)

mu_a = zeros(1, 5);

for i = 1:5
mu_a(i) = exp(-(phi - c_phi(i))^2 / (2 * w_phi^2));
end

mu_attv = zeros(1, 6);

for i = 1:6
mu_attv(i) = exp(-(phi_dot - c_phi_dot(i))^2 / (2 * w_phi_dot^2));
end

zeta_a = zeros(30, 1);
k= 1;

for i = 1:5
    for j = 1:6
    zeta_a(k) = mu_a(i) * mu_attv(j);
    k = k + 1;
    end
end
end
