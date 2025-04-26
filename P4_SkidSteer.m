% Simulacion Skid-Steer 4 ruedas con friccion LuGre
clear; clc; close all;

% Parametros del robot
m = 50;          % Masa total (kg)
Iz = 10;         % Momento de inercia (kg*m^2)
r = 0.1;         % Radio de las ruedas (m)
b = 0.3;         % Mitad de la distancia entre ruedas (m)

M = diag([m, m, Iz]); % Matriz de inercia

% Parametros LuGre 
sigma_0 = 100;   % sigma 0
sigma_1 = 1;     % sigma 1
sigma_2 = 0.5;   % sigma 2
Fs = 30;         % Friccion estatica
Fc = 18;         % Friccion dinamica
vs = 0.01;       

% Tiempo de simulacion
dt = 0.01;         % Paso de integracion (s)
T_total = 12.4;      % Tiempo total (s)
t = 0:dt:T_total;

% Estado inicial
q = [0; 0; 0];      % [X; Y; theta]
u = [0; 0];         % [U; omega]

z_r = 0;           % Estado interno rueda derecha
z_l = 0;           % Estado interno rueda izquierda

% Perfil de torque de las ruedas
Tau_r = zeros(1, length(t));
Tau_l = zeros(1, length(t));

Tau_r(t < 0.1) = 0.5;        % Aceleracion hasta 1 segundo
Tau_l(t < 3) = 0;
Tau_r(t >= 3 & t < 8) = 0; % Desaceleracion entre 1 y 2 segundos
Tau_l(t >= 3 & t < 8) = 0;

% Almacenamiento para graficar
Q = zeros(3,length(t));
U = zeros(2,length(t));

%% Loop de simulacion
for k = 1:length(t)

    theta = q(3);

    % Matriz N(q)
    N = [cos(theta) 0;
         sin(theta) 0;
         0          1];

    % q_dot
    q_dot = N*u;

    % Velocidades longitudinal y rotacional
    Uq = cos(theta)*q_dot(1) + sin(theta)*q_dot(2);
    Vq = -sin(theta)*q_dot(1) + cos(theta)*q_dot(2);
    omega = u(2);

    % Velocidades relativas ruedas
    v_r = Uq + b*omega;
    v_l = Uq - b*omega;

    % Actualizacion de estados internos LuGre
    g_r = Fc + (Fs - Fc)*exp(-(v_r/vs)^2);
    g_l = Fc + (Fs - Fc)*exp(-(v_l/vs)^2);

    dz_r = v_r - (sigma_0/g_r)*z_r;
    dz_l = v_l - (sigma_0/g_l)*z_l;

    z_r = z_r + dz_r*dt;
    z_l = z_l + dz_l*dt;

    % Fuerzas de friccion
    F_r = sigma_0*z_r + sigma_1*dz_r + sigma_2*v_r;
    F_l = sigma_0*z_l + sigma_1*dz_l + sigma_2*v_l;

    % Fuerza neta y momento
    F_x = F_r + F_l;
    M_z = b*(F_r - F_l);

    % Vector de friccion ruedas
    f_fric = [-F_x*cos(theta); -F_x*sin(theta); -M_z];

    % Matriz de termino extra
    Cq_dot = [m*Vq*omega;
             -m*Uq*omega;
              0];

    % Matriz B(q)
    B = (1/r)*[cos(theta) cos(theta);
               sin(theta) sin(theta);
               b           -b];

    % Torques actuales
    tau = [Tau_r(k); Tau_l(k)];

    % Matrices proyectadas
    M_u = N'*M*N;
    C_u = N'*Cq_dot;
    f_u = N'*f_fric;
    B_u = N'*B;

    % Dinamica proyectada
    dot_u = M_u\(B_u*tau - C_u - f_u);

    % Integracion
    u = u + dot_u*dt;
    q = q + q_dot*dt;

    % Almacenar
    Q(:,k) = q;
    U(:,k) = u;

end

%% Graficos
figure;
subplot(2,1,1);
plot(t, U(1,:), 'LineWidth', 1.5);
ylabel('Velocidad longitudinal U [m/s]'); grid on;
title('Velocidades del Skid-Steer');

subplot(2,1,2);
plot(t, U(2,:), 'LineWidth', 1.5);
ylabel('Velocidad angular \omega [rad/s]');
xlabel('Tiempo [s]'); grid on;

%% Animacion del skid-steer
L = 0.6; % largo del chasis (m)
W = 0.4; % ancho del chasis (m)

figure;
hold on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); grid on;
title('Animacion Skid-Steer');

for k = 1:10:length(t)
    clf;
    hold on; axis equal;
    plot(Q(1,1:k), Q(2,1:k), 'r--', 'LineWidth', 1.2); % trayectoria recorrida
    
    xlim([min(Q(1,1:k))-1, max(Q(1,1:k))+1]);
    ylim([min(Q(2,1:k))-2, max(Q(2,1:k))+2]);
    xlabel('X [m]'); ylabel('Y [m]');
    title('Animacion Skid-Steer');

    % Posicion y orientacion actual
    x_c = Q(1,k);
    y_c = Q(2,k);
    theta = Q(3,k);

    % Chasis
    corners = [-L/2, -W/2;
                L/2, -W/2;
                L/2,  W/2;
               -L/2,  W/2]';

    R = [cos(theta) -sin(theta);
         sin(theta)  cos(theta)];

    chasis = R*corners + [x_c; y_c];
    fill(chasis(1,:), chasis(2,:), 'c');

    % Dibujar ruedas
    wheel_L = 0.2; % largo rueda
    wheel_W = 0.05; % ancho rueda

    wheel_pos = [L/2, W/2;
                 -L/2, W/2;
                 L/2, -W/2;
                 -L/2, -W/2];

    for i = 1:4
        wp = wheel_pos(i,:)';
        wheel = [-wheel_L/2, -wheel_W/2;
                  wheel_L/2, -wheel_W/2;
                  wheel_L/2,  wheel_W/2;
                 -wheel_L/2,  wheel_W/2]';

        wheel_rot = R*wheel + (R*wp + [x_c; y_c]);
        fill(wheel_rot(1,:), wheel_rot(2,:), 'k');
    end

    pause(0.05);
end