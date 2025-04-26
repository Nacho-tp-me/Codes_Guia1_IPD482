%% Simulación 2-Trailer sin ode45, integración explícita y ruedas
clear; close all; clc

% Parámetros físicos [m]
L1 = 1.2925;   % tractor → trailer1
L2 = 1.055;   % trailer1 → trailer2
Lt = 0.99;   Wt = 0.67;   % Tractor Husky A200
Lr = 0.515;  Wr = 0.457;  % Trailer

% Perfil temporal y de control
dt = 0.08;               % paso de integración
T1 = 45; 
T2 = 0;         % línea recta, luego curva
T  = T1 + T2;
t  = 0:dt:T;
N  = numel(t);        %n de muestras

v = 0.2*ones(1,N);   % 0.2 m/s siempre
omega = zeros(1,N);
omega(t> T1) = 0.1;      % 0 antes de T1, 0.3 rad/s tras T1
omega(t> T2) = 0;      % 0 antes de T1, 0.3 rad/s tras T1

%% Vectores de estado
x0  = zeros(1,N);   
y0  = zeros(1,N); 
th0 = zeros(1,N);
th1 = zeros(1,N);   
th2 = zeros(1,N);

% Condición inicial (t=0)
x0(1)=0; 
y0(1)=0; 
th0(1)=0;
th1(1)=0; 
th2(1)=0;

%% Integración explícita (Euler)
for k = 1:N-1
  % Cinemática del tractor
  x0(k+1)  = x0(k)  + v(k)*cos(th0(k)) * dt;
  y0(k+1)  = y0(k)  + v(k)*sin(th0(k)) * dt;
  th0(k+1) = th0(k) + omega(k)*dt;

  % Giro del primer trailer
  th1(k+1) = th1(k) + (v(k)/L1)*sin(th0(k)-th1(k))*dt;
  % Giro del segundo trailer
  th2(k+1) = th2(k) + (v(k)*cos(th0(k)-th1(k))/L2)*sin(th1(k)-th2(k))*dt;
end

%% Posiciones de trailers
x1 = x0 - L1*cos(th1);
y1 = y0 - L1*sin(th1);
x2 = x1 - L2*cos(th2);
y2 = y1 - L2*sin(th2);

%% --- Animación ---
figure('Color','w','Position',[100 100 700 500]);
axis equal; grid on; hold on;
xlabel('x [m]'); ylabel('y [m]');
title('Trayectoria 2-Trailers');

for k = 1:5:N
  cla;
  % trayectorias
  plot(x0(1:k),y0(1:k),'-b','LineWidth',1); hold on;
  plot(x1(1:k),y1(1:k),'-r','LineWidth',1);
  plot(x2(1:k),y2(1:k),'-g','LineWidth',1);

  % uniones entre ejes
  plot([x0(k), x1(k)], [y0(k), y1(k)], '-k','LineWidth',2);
  plot([x1(k), x2(k)], [y1(k), y2(k)], '-k','LineWidth',2)

  % dibujo cuerpos + ruedas
  drawRectWheels(x0(k),y0(k),Lt,Wt,th0(k),'b');
  drawRectWheels(x1(k),y1(k),Lr,Wr,th1(k),'r');
  drawRectWheels(x2(k),y2(k),Lr,Wr,th2(k),'g');

  legend('Tractor','Trailer1','Trailer2','Location','Best');
  drawnow;
  pause(0.001);
end

% función dibujar carros + 2 ruedas
function drawRectWheels(xc,yc,L,W,theta,col)
wr = 0.1;  % ancho rueda
hr = 0.04;  % grosor rueda
  % vértices de rectángulo centrado en 0
  V = [ L/2,  W/2;
       -L/2,  W/2;
       -L/2, -W/2;
        L/2, -W/2 ]';

  R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
  P = R*V + [xc;yc];  % rotar y trasladar

  patch(P(1,:),P(2,:),col,'FaceAlpha',.3,'EdgeColor',col,'LineWidth',1.5);

  % calcular centros de ejes (del/trasero, izquierda/derecha)
  offs = W/2*0.6;
  centers = R*[[L/2; offs],[-L/2; offs],[L/2; -offs],[-L/2; -offs]] + [xc;yc];
  % dibujar ruedas
% calcular centros de ejes (del/trasero, izquierda/derecha)
offs    = W/2*0.6;  
centers = [ 0,  offs;
           0,  -offs]';   % 2×4
centers = R*centers + [xc; yc];  % rotar y trasladar

% para cada rueda, dibuja un rectángulo de ancho wr y alto hr
for i = 1:2
  cx = centers(1,i);
  cy = centers(2,i);
  % vértices de un rectángulo centrado en (0,0)
  Vr = [ -wr/2, -hr/2;
          wr/2, -hr/2;
          wr/2,  hr/2;
         -wr/2,  hr/2 ]';
  % lo giramos con R (ya es misma R que el cuerpo)
  Pr = R*Vr + [cx; cy];
  patch( Pr(1,:), Pr(2,:), col, 'FaceAlpha',1, 'EdgeColor',col );
end
end
