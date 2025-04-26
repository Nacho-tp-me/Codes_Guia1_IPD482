clc; clear; close all;

% Parámetros simbólicos
syms b L beta r real                 % parámetros del robot
syms phi_T_dot phi_F_dot real       % entradas de control (ruedas)
syms x_dot y_dot theta_dot theta real  % velocidades del robot

A = [... 
    1, 0, -b;                                  % rodadura trasera derecha
    0, 1,  0;                                  % lateral trasera derecha
    1, 0,  b;                                  % rodadura trasera izquierda
    0, 1,  0;                                  % lateral trasera izquierda
    sin(beta), -cos(beta), -L*cos(beta);       % rodadura delantera
    cos(beta),  sin(beta),  L*sin(beta)        % lateral delantera
];

B = [...
    r, 0;
    0, 0;
    r, 0;
    0, 0;
    0, r;
    0, 0
];

phi_dot = [phi_T_dot; phi_F_dot];

% Matriz de rotación
R = [cos(theta), sin(theta), 0;
    -sin(theta), cos(theta), 0;
     0, 0, 1];

% Pseudo-inversa
R_theta_0 = subs(R, theta, 0);                     % evaluar con theta = 0
xi_dot_local = simplify(inv(A.'*A)*A.' * B);
xi_dot_global = simplify(inv(R_theta_0) * xi_dot_local);  % transformar al global

% ---------- 7. Mostrar resultado ----------
disp('Matriz cinemática externa:');
pretty(xi_dot_global)
