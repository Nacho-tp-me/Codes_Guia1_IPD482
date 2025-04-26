clc; clear;

% Parámetros simbólicos
syms r b L beta phi_T_dot phi_F_dot real
syms x_dot y_dot theta_dot real

% Matriz inicial con phi_F_dot
M_inicial = [ (2*r)/3, (r/3)*sin(beta);
              0, -b^2*r*cos(beta)/(L^2 + 3*b^2);
              0, -L*r*cos(beta)/(L^2 + 3*b^2)];

% Relación para phi_F_dot obtenida de la restricción de rodadura
relacion_phiF = (1/r)*(x_dot*sin(beta) - y_dot*cos(beta) - L*theta_dot*cos(beta));

% Vector original del sistema
vector_original = M_inicial*[phi_T_dot; phi_F_dot];

% Resolver sistema simbólico explícitamente (solución estructurada)
solucion = solve([...
    x_dot == vector_original(1),...
    y_dot == vector_original(2),...
    theta_dot == vector_original(3),...
    phi_F_dot == relacion_phiF],...
    [x_dot, y_dot, theta_dot, phi_F_dot],'ReturnConditions',true);

% Tomar solución única explícita
x_dot_sol = simplify(solucion.x_dot);
y_dot_sol = simplify(solucion.y_dot);
theta_dot_sol = simplify(solucion.theta_dot);

% Matriz final dependiente solo de phi_T_dot y beta
M_final = simplify([x_dot_sol; y_dot_sol; theta_dot_sol],'Steps',500);

% Mostrar resultado final más simplificado
pretty(M_final)
