% Parámetros del robot
r = 0.05;         % Radio de cada rueda
R_robot = 0.15;   % "Radio" del centro del robot a cada rueda 
n = 5;            % Número de ruedas

% Ángulos de las ruedas 72 grados separación
theta_i = zeros(n,1);
for i = 1:n
    theta_i(i) = (i-1) * 2*pi/n;  % en radianes
end

% perfil de velocidades:
u = [0.3; 0.0; 0.5];  % [vx, vy, omega] en marco global

% Matriz de rotación (global -> robot)
R_matrix = @(th) [ cos(th),  sin(th), 0;
                  -sin(th),  cos(th), 0;
                   0,        0,       1];

% Construir la matriz J1: fila i -> [ sin(θ_i), -cos(θ_i), -R_robot*cos(θ_i) ]
J1 = zeros(n, 3);
for i = 1:n
    angle = theta_i(i);
    J1(i,1) = sin(angle);
    J1(i,2) = -cos(angle);
    J1(i,3) = -R_robot * cos(angle);
end

%% Simulación
dt = 0.05;  % Paso de integración
T  = 10;    % Tiempo total (s)
steps = round(T/dt)

% Pose inicial del robot en el marco global 
x = 0;  y = 0;  theta = 0;

% Para almacenar resultados
pose_history = zeros(steps,3);
wheel_speed_history = zeros(steps,n);

figure('Name','Simulacion Robot 5 Ruedas Suecas');
axis equal; grid on; hold on;
% Limites segun el movimiento esperado (puedes ajustarlos)
xlim([-1,  T*u(1) + 1]);
ylim([-1,  1]);
xlabel('x (m)'); ylabel('y (m)');
title('Trayectoria del robot');

% Graficar trayectoria (por ahora, vacía)
traj_plot = plot(0,0,'b-','LineWidth',2);

% Definir puntos para dibujar el círculo del robot
N_circ = 50;                 % número de puntos para aproximar el círculo
ang_circ = linspace(0,2*pi,N_circ);
circle_x = R_robot * cos(ang_circ);
circle_y = R_robot * sin(ang_circ);

% Definir línea que indique la orientación (flecha roja)
heading_x = [0, R_robot];
heading_y = [0, 0];

% Dibujar las primitivas iniciales
robot_circle_plot = plot(circle_x, circle_y,'r-','LineWidth',2);   % círculo
robot_heading_plot = plot(heading_x, heading_y,'r-','LineWidth',2);% flecha
%% Bucle de simulación
for k = 1:steps
    % 1) Actualizar pose (integramos la velocidad en marco global)
    x = x + u(1)*dt;
    y = y + u(2)*dt;
    theta = theta + u(3)*dt;
    
    pose_history(k,:) = [x, y, theta];
    
    % 2) Calcular las velocidades angulares de las ruedas
    R_theta = R_matrix(theta);  % rotación global->robot
    dot_phi = (1/r) * (J1 * (R_theta * u));
    wheel_speed_history(k,:) = dot_phi';
    
    % 3) Actualizar la gráfica de la trayectoria
    set(traj_plot, 'XData', pose_history(1:k,1), 'YData', pose_history(1:k,2));
    
    % 4) Transformar círculo y flecha al marco global del instante actual
    %    (usando la orientación theta y posición [x, y])
    %    Tomamos la parte 2x2 de R_theta y le sumamos el offset [x;y]
    
    % Círculo
    circ_global = R_theta(1:2,1:2) * [circle_x; circle_y] + [x; y]; 
    set(robot_circle_plot,'XData',circ_global(1,:),'YData',circ_global(2,:));
    
    % Flecha de orientación
    heading_global = R_theta(1:2,1:2)*[heading_x; heading_y] + [x; y];
    set(robot_heading_plot,'XData',heading_global(1,:),'YData',heading_global(2,:));
    
    drawnow;
    pause(dt*0.5);  % Ajusta para controlar la velocidad de animación
end

%% Graficar velocidades de las ruedas (opcional)
figure('Name','Velocidades de Ruedas');
time = (0:steps-1)*dt;
plot(time, wheel_speed_history,'LineWidth',2);
xlabel('Tiempo (s)');
ylabel('Vel. angular (rad/s)');
grid on; legend('R1','R2','R3','R4','R5');
title('Velocidades Angulares de las 5 Ruedas Suecas');
